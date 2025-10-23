#include "abclib/hardware/advanced_motor.hpp"
#include <cmath>
#include <algorithm>
#include <mutex>
namespace abclib::hardware
{

    AdvancedMotor::AdvancedMotor(
        pros::Motor *base_motor,
        const MotorConfig &motor_config,
        double gearing,
        bool enable_feedforward)
        : motor_(base_motor),
          rotation_sensor_(motor_config.rotation),
          position_pid_(abclib::control::PIDConstants{
              motor_config.kPs,
              motor_config.kIs,
              motor_config.kDs}),
          velocity_pid_(abclib::control::PIDConstants{
              motor_config.kPv,
              motor_config.kIv,
              motor_config.kDv}),
          config_(motor_config),
          ticks_(find_ticks(base_motor, gearing)),
          gearing_(gearing),
          using_encoder_(rotation_sensor_ != nullptr),
          use_feedforward_(enable_feedforward)
    {
        set_brake_mode(motor_config.brake_mode);
    }

    AdvancedMotor::AdvancedMotor(
        int8_t port,
        pros::MotorGearset gearset,
        const MotorConfig &motor_config,
        double gearing,
        bool enable_feedforward)
        : owned_motor_(pros::Motor(port, gearset)), // Create and own the motor
          motor_(&owned_motor_.value()),            // Point to our owned motor
          rotation_sensor_(motor_config.rotation),
          position_pid_(control::PIDConstants{
              motor_config.kPs,
              motor_config.kIs,
              motor_config.kDs,
              motor_config.max_integral_position}),
          velocity_pid_(control::PIDConstants{
              motor_config.kPv,
              motor_config.kIv,
              motor_config.kDv,
              motor_config.max_integral_velocity}),
          config_(motor_config),
          ticks_(find_ticks(&owned_motor_.value(), gearing)),
          gearing_(gearing),
          using_encoder_(rotation_sensor_ != nullptr),
          use_feedforward_(enable_feedforward)
    {
        set_brake_mode(motor_config.brake_mode);
    }

    AdvancedMotor::~AdvancedMotor()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (current_task_.has_value())
        {
            current_task_->notify();
        }
    }

    // static
    double AdvancedMotor::find_ticks(pros::Motor *motor, double further_gearing)
    {
        switch (motor->get_gearing())
        {
        case pros::MotorGearset::blue:
            return 300 * further_gearing;
        case pros::MotorGearset::green:
            return 900 * further_gearing;
        case pros::MotorGearset::red:
            return 1800 * further_gearing;
        }
        return 0;
    }

    void AdvancedMotor::reset_position()
    {
        motor_->tare_position();
        if (using_encoder_)
            rotation_sensor_->reset_position();
    }

    double AdvancedMotor::get_ticks() const
    {
        return ticks_;
    }

    units::Current AdvancedMotor::get_current_draw() const
    {
        return units::Current::from_milliamps(motor_->get_current_draw());
    }

    void AdvancedMotor::move_voltage(units::Voltage voltage)
    {
        motor_->move(voltage.to_pros_units());
    }

    void AdvancedMotor::brake()
    {
        motor_->brake();
    }

    void AdvancedMotor::set_brake_mode(pros::motor_brake_mode_e_t brake_mode)
    {
        motor_->set_brake_mode(brake_mode);
    }

    units::MotorAngularVelocity AdvancedMotor::get_raw_velocity() const
    {
        if (using_encoder_)
        {
            // Rotation sensor returns centidegrees/s
            // Convert: centideg/s -> deg/s -> rad/s
            double deg_per_sec = rotation_sensor_->get_velocity() / 100.0;
            double rad_per_sec = units::Degrees(deg_per_sec).to_radians().value;
            return units::MotorAngularVelocity(rad_per_sec);
        }
        else
        {
            // Motor returns RPM, convert to rad/s
            return units::MotorAngularVelocity::from_rpm(motor_->get_actual_velocity());
        }
    }

    units::MotorPosition AdvancedMotor::get_raw_position() const
    {
        if (using_encoder_)
        {
            // Rotation sensor returns centidegrees
            // Convert: centideg -> deg -> rad
            double centidegrees = rotation_sensor_->get_position();
            double degrees = centidegrees / 100.0;
            return units::MotorPosition(units::Degrees(degrees));
        }
        else
        {
            // Motor returns degrees
            double degrees = motor_->get_position();
            return units::MotorPosition(units::Degrees(degrees));
        }
    }

    units::Degrees AdvancedMotor::get_position() const
    {
        return get_raw_position().angle.to_degrees();
    }

    void AdvancedMotor::rotate_to(units::Degrees target, units::Time timeout,
                                  units::Voltage min_voltage, units::Voltage max_voltage)
    {
        reset_position();
        uint32_t start = pros::millis();
        const units::Degrees threshold(1.0);

        const double dt = 0.01;
        position_pid_.reset();

        while ((pros::millis() - start) < timeout.to_millis_uint())
        {
            // Check if we should stop (for task cancellation)
            if (pros::Task::notify_take(true, 0) > 0)
            {
                break;
            }

            units::Degrees pos = get_position();
            units::MotorAngularVelocity vel = get_raw_velocity();

            if (std::fabs((target - pos).value) <= threshold.value &&
                std::fabs(vel.to_rpm()) < 5)
                break;

            double err = (target - pos).value;
            double out = position_pid_.compute(err, dt);

            units::Voltage output_voltage(out);
            output_voltage = units::Voltage(std::clamp(output_voltage.volts, min_voltage.volts, max_voltage.volts));
            move_voltage(output_voltage);
            pros::delay(10);
        }
        brake();
    }

    void AdvancedMotor::rotate_to_task(units::Degrees target, units::Time timeout,
                                       units::Voltage min_voltage, units::Voltage max_voltage)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        // Signal existing task to stop
        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        // Create new task
        current_task_ = pros::Task([this, target, timeout, min_voltage, max_voltage]()
                                   { this->rotate_to(target, timeout, min_voltage, max_voltage); });
    }

    void AdvancedMotor::hold_velocity(units::RPM target_rpm, units::Time timeout,
                                      units::Voltage min_voltage, units::Voltage max_voltage)
    {
        uint32_t start = pros::millis();
        const double dt = 0.01;
        velocity_pid_.reset();

        units::MotorAngularVelocity target_velocity = units::MotorAngularVelocity::from_rpm(target_rpm.value);

        while ((pros::millis() - start) < timeout.to_millis_uint())
        {
            if (pros::Task::notify_take(true, 0) > 0)
            {
                break;
            }

            units::MotorAngularVelocity vel = get_raw_velocity();

            double err = target_velocity.rad_per_sec - vel.rad_per_sec;
            double out = velocity_pid_.compute(err, dt);

            if (use_feedforward_)
            {
                double ff = config_.kS * ((target_velocity.rad_per_sec > 0) - (target_velocity.rad_per_sec < 0));
                ff += config_.kV * target_velocity.rad_per_sec;
                out += ff;
            }

            units::Voltage output_voltage(out);
            output_voltage = units::Voltage(std::clamp(output_voltage.volts, min_voltage.volts, max_voltage.volts));
            move_voltage(output_voltage);
            pros::delay(10);
        }
    }

    void AdvancedMotor::hold_velocity_task(units::RPM target_rpm, units::Time timeout,
                                           units::Voltage min_voltage, units::Voltage max_voltage)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        // Signal existing task to stop
        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        // Create new task
        current_task_ = pros::Task([this, target_rpm, timeout, min_voltage, max_voltage]()
                                   { this->hold_velocity(target_rpm, timeout, min_voltage, max_voltage); });
    }

    void AdvancedMotor::move_velocity_continuous(units::MotorAngularVelocity target_velocity)
    {
        const double dt = 0.01; // Assumes 100Hz calls

        units::MotorAngularVelocity vel = get_raw_velocity();
        double err = target_velocity.rad_per_sec - vel.rad_per_sec;
        double out = velocity_pid_.compute(err, dt);

        // Add feedforward if enabled
        if (use_feedforward_)
        {
            double ff = config_.kS * ((target_velocity.rad_per_sec > 0) - (target_velocity.rad_per_sec < 0));
            ff += config_.kV * target_velocity.rad_per_sec;
            out += ff;
        }

        units::Voltage output_voltage(out);
        output_voltage = units::Voltage(std::clamp(output_voltage.volts, -12.0, 12.0));
        move_voltage(output_voltage);
    }

    void AdvancedMotor::move_velocity_continuous_task(units::MotorAngularVelocity target_velocity)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        // Signal existing task to stop
        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        // Create new task that maintains velocity indefinitely
        current_task_ = pros::Task([this, target_velocity]()
                                   {
        while (pros::Task::notify_take(true, 0) == 0) {
            this->move_velocity_continuous(target_velocity);
            pros::delay(10);
        }
        this->brake(); });
    }

    void AdvancedMotor::set_feedforward(bool enable)
    {
        use_feedforward_ = enable;
    }

    bool AdvancedMotor::is_using_feedforward() const
    {
        return use_feedforward_;
    }

    units::RPM AdvancedMotor::get_velocity() const
    {
        return units::RPM::from_rad_per_sec(get_raw_velocity().rad_per_sec);
    }

    void AdvancedMotor::move_velocity_continuous(units::MotorAngularVelocity target_velocity,
                                                 double override_kS,
                                                 double override_kV)
    {
        const double dt = 0.01; // Assumes 100Hz calls

        units::MotorAngularVelocity vel = get_raw_velocity();
        double err = target_velocity.rad_per_sec - vel.rad_per_sec;
        double out = velocity_pid_.compute(err, dt);

        // Use override constants instead of config_
        if (use_feedforward_)
        {
            double ff = override_kS * ((target_velocity.rad_per_sec > 0) - (target_velocity.rad_per_sec < 0));
            ff += override_kV * target_velocity.rad_per_sec;
            out += ff;
        }

        units::Voltage output_voltage(out);
        output_voltage = units::Voltage(std::clamp(output_voltage.volts, -12.0, 12.0));
        move_voltage(output_voltage);
    }

}