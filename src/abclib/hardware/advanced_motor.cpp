#include "abclib/hardware/advanced_motor.hpp"
#include <cmath>
#include <algorithm>
#include <mutex>
#include "abclib/telemetry/telemetry.hpp"

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
        double compensation_scale = 1.0;
        bool compensation_active = false;

        if (config_.enable_voltage_compensation)
        {
            units::Voltage battery = units::Voltage::from_millivolts(pros::battery::get_voltage());

            if (battery < config_.compensation_min_battery)
            {
                compensation_scale = config_.compensation_nominal.to_volts() / battery.to_volts();
                voltage = voltage * compensation_scale;
                compensation_active = true;
            }
        }

        auto &data = telemetry::g_telemetry.get_write_buffer();
        data.voltage_compensation_active = compensation_active;
        data.voltage_compensation_scale = compensation_scale;

        motor_->move(voltage.to_volts() * 1000.0); // PROS uses millivolts
    }

    void AdvancedMotor::brake()
    {
        motor_->brake();
    }

    void AdvancedMotor::set_brake_mode(pros::motor_brake_mode_e_t brake_mode)
    {
        motor_->set_brake_mode(brake_mode);
    }

    units::AngularVelocity AdvancedMotor::get_raw_velocity() const
    {
        if (using_encoder_)
        {
            // Rotation sensor returns centidegrees/s
            // Convert: centideg/s -> deg/s -> rad/s
            double deg_per_sec = rotation_sensor_->get_velocity() / 100.0;
            return units::AngularVelocity::from_deg_per_sec(deg_per_sec);
        }
        else
        {
            // Motor returns RPM, convert to rad/s
            return units::AngularVelocity::from_rpm(motor_->get_actual_velocity());
        }
    }

    units::Angle AdvancedMotor::get_raw_position() const
    {
        if (using_encoder_)
        {
            // Rotation sensor returns centidegrees
            // Convert: centideg -> deg -> rad
            double centidegrees = rotation_sensor_->get_position();
            double degrees = centidegrees / 100.0;
            return units::Angle::from_degrees(degrees);
        }
        else
        {
            // Motor returns degrees
            double degrees = motor_->get_position();
            return units::Angle::from_degrees(degrees);
        }
    }

    units::Angle AdvancedMotor::get_position() const
    {
        return get_raw_position();
    }

    void AdvancedMotor::rotate_to(units::Angle target, units::Time timeout,
                                  units::Voltage min_voltage, units::Voltage max_voltage)
    {
        reset_position();
        uint32_t start = pros::millis();
        units::Angle threshold = units::Angle::from_degrees(1.0);

        const double dt = 0.01;
        position_pid_.reset();

        while ((pros::millis() - start) < timeout.to_milliseconds())
        {
            // Check if we should stop (for task cancellation)
            if (pros::Task::notify_take(true, 0) > 0)
            {
                break;
            }

            units::Angle pos = get_position();
            units::AngularVelocity vel = get_raw_velocity();

            // Convert the subtraction result back to Angle type
            units::Angle error(target - pos);

            if (std::fabs(error.to_degrees()) <= threshold.to_degrees() &&
                std::fabs(vel.to_rpm()) < 5)
                break;

            double err = error.to_degrees();
            double out = position_pid_.compute(err, dt);

            units::Voltage output_voltage = units::Voltage::from_volts(out);
            output_voltage = units::Voltage::from_volts(
                std::clamp(output_voltage.to_volts(), min_voltage.to_volts(), max_voltage.to_volts()));
            move_voltage(output_voltage);
            pros::delay(10);
        }
        brake();
    }

    void AdvancedMotor::rotate_to_task(units::Angle target, units::Time timeout,
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

    void AdvancedMotor::hold_velocity(units::AngularVelocity target_vel, units::Time timeout,
                                      units::Voltage min_voltage, units::Voltage max_voltage)
    {
        uint32_t start = pros::millis();
        const double dt = 0.01;
        velocity_pid_.reset();

        while ((pros::millis() - start) < timeout.to_milliseconds())
        {
            if (pros::Task::notify_take(true, 0) > 0)
            {
                break;
            }

            units::AngularVelocity vel = get_raw_velocity();

            double err = target_vel.to_rad_per_sec() - vel.to_rad_per_sec();
            double out = velocity_pid_.compute(err, dt);

            if (use_feedforward_)
            {
                double ff = config_.kS * ((target_vel.to_rad_per_sec() > 0) - (target_vel.to_rad_per_sec() < 0));
                ff += config_.kV * target_vel.to_rad_per_sec();
                out += ff;
            }

            units::Voltage output_voltage = units::Voltage::from_volts(out);
            output_voltage = units::Voltage::from_volts(
                std::clamp(output_voltage.to_volts(), min_voltage.to_volts(), max_voltage.to_volts()));
            move_voltage(output_voltage);
            pros::delay(10);
        }
    }

    void AdvancedMotor::hold_velocity_task(units::AngularVelocity target_vel, units::Time timeout,
                                           units::Voltage min_voltage, units::Voltage max_voltage)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        // Signal existing task to stop
        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        // Create new task
        current_task_ = pros::Task([this, target_vel, timeout, min_voltage, max_voltage]()
                                   { this->hold_velocity(target_vel, timeout, min_voltage, max_voltage); });
    }

    void AdvancedMotor::move_velocity_continuous(units::AngularVelocity target_velocity)
    {
        const double dt = 0.01; // Assumes 100Hz calls

        units::AngularVelocity vel = get_raw_velocity();
        double err = target_velocity.to_rad_per_sec() - vel.to_rad_per_sec();
        double out = velocity_pid_.compute(err, dt);

        // Add feedforward if enabled
        if (use_feedforward_)
        {
            double ff = config_.kS * ((target_velocity.to_rad_per_sec() > 0) - (target_velocity.to_rad_per_sec() < 0));
            ff += config_.kV * target_velocity.to_rad_per_sec();
            out += ff;
        }

        units::Voltage output_voltage = units::Voltage::from_volts(out);
        output_voltage = units::Voltage::from_volts(
            std::clamp(output_voltage.to_volts(), -12.0, 12.0));
        move_voltage(output_voltage);
    }

    void AdvancedMotor::move_velocity_continuous_task(units::AngularVelocity target_velocity)
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

    units::AngularVelocity AdvancedMotor::get_velocity() const
    {
        return get_raw_velocity();
    }

    void AdvancedMotor::move_velocity_continuous(units::AngularVelocity target_velocity,
                                                 double override_kS,
                                                 double override_kV)
    {
        const double dt = 0.01; // Assumes 100Hz calls

        units::AngularVelocity vel = get_raw_velocity();
        double err = target_velocity.to_rad_per_sec() - vel.to_rad_per_sec();
        double out = velocity_pid_.compute(err, dt);

        // Use override constants instead of config_
        if (use_feedforward_)
        {
            double ff = override_kS * ((target_velocity.to_rad_per_sec() > 0) - (target_velocity.to_rad_per_sec() < 0));
            ff += override_kV * target_velocity.to_rad_per_sec();
            out += ff;
        }

        units::Voltage output_voltage = units::Voltage::from_volts(out);
        output_voltage = units::Voltage::from_volts(
            std::clamp(output_voltage.to_volts(), -12.0, 12.0));
        move_voltage(output_voltage);
    }

    void AdvancedMotor::move_velocity_pros(units::AngularVelocity target_velocity)
    {
        // Convert rad/s to RPM for PROS
        double target_rpm = target_velocity.to_rpm();
        motor_->move_velocity(target_rpm);
    }

    void AdvancedMotor::move_velocity_pros_task(units::AngularVelocity target_velocity)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        // Signal existing task to stop
        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        // Create new task that maintains velocity indefinitely using PROS controller
        current_task_ = pros::Task([this, target_velocity]()
                                   {
        while (pros::Task::notify_take(true, 0) == 0) {
            this->move_velocity_pros(target_velocity);
            pros::delay(10);
        }
        this->brake(); });
    }

}