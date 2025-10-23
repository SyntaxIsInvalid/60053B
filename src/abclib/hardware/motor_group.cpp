#include "abclib/hardware/motor_group.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>
#include "abclib/units/units.hpp"
namespace abclib::hardware
{

    AdvancedMotorGroup::AdvancedMotorGroup(
        const std::vector<AdvancedMotor *> &motors_list,
        const motor_group_config &config,
        bool enable_feedforward)
        : motors(motors_list),
          rotation_sensor(config.rotation),
          further_gearing(config.group_gearing),
          group_config(config),
          using_encoder(config.rotation != nullptr),
          position_pid(control::PIDConstants{config.kPs, config.kIs, config.kDs, config.max_integral_position}),
          velocity_pid(control::PIDConstants{config.kPv, config.kIv, config.kDv, config.max_integral_velocity}),
          use_feedforward(enable_feedforward)
    {
        if (!motors.empty())
        {
            ticks = motors[0]->get_ticks() * further_gearing;
        }
        else
        {
            ticks = 0;
        }
    }
    AdvancedMotorGroup::AdvancedMotorGroup(
        const std::vector<int8_t> &ports,
        pros::MotorGearset gearset,
        const motor_group_config &config,
        bool enable_feedforward)
        : rotation_sensor(config.rotation),
          further_gearing(config.group_gearing),
          group_config(config),
          using_encoder(config.rotation != nullptr),
          position_pid(control::PIDConstants{config.kPs, config.kIs, config.kDs, config.max_integral_position}),
          velocity_pid(control::PIDConstants{config.kPv, config.kIv, config.kDv, config.max_integral_velocity}),
          use_feedforward(enable_feedforward)
    {
        owned_motors_.reserve(ports.size());
        owned_advanced_motors_.reserve(ports.size());
        motors.reserve(ports.size());

        for (int8_t port : ports)
        {
            // Create the base motor with unique_ptr
            owned_motors_.push_back(std::make_unique<pros::Motor>(port, gearset));

            // Create empty motor config
            MotorConfig motor_cfg{};
            motor_cfg.brake_mode = pros::E_MOTOR_BRAKE_BRAKE;

            // Create the advanced motor wrapper with unique_ptr
            owned_advanced_motors_.push_back(
                std::make_unique<AdvancedMotor>(owned_motors_.back().get(), motor_cfg));

            // Store raw pointer for existing interface
            motors.push_back(owned_advanced_motors_.back().get());
        }

        if (!motors.empty())
        {
            ticks = motors[0]->get_ticks() * further_gearing;
        }
        else
        {
            ticks = 0;
        }
    }

    AdvancedMotorGroup::~AdvancedMotorGroup()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (current_task_.has_value())
        {
            current_task_->notify();
        }
    }

    void AdvancedMotorGroup::reset_position()
    {
        for (auto m : motors)
            m->reset_position();
        if (using_encoder)
            rotation_sensor->reset_position();
    }

    void AdvancedMotorGroup::move_voltage(units::Voltage voltage)
    {
        for (auto m : motors)
            m->move_voltage(voltage);
    }

    void AdvancedMotorGroup::set_brake_mode(pros::motor_brake_mode_e_t mode)
    {
        for (auto m : motors)
            m->set_brake_mode(mode);
    }

    void AdvancedMotorGroup::brake()
    {
        for (auto m : motors)
            m->brake();
    }

    double AdvancedMotorGroup::get_ticks()
    {
        return ticks;
    }

    units::MotorAngularVelocity AdvancedMotorGroup::get_raw_velocity() const
    {
        if (using_encoder)
        {
            // Rotation sensor returns centidegrees/s, convert to rad/s
            // centideg/s -> deg/s -> rad/s
            double deg_per_sec = rotation_sensor->get_velocity() / 100.0;
            double rad_per_sec = units::Degrees(deg_per_sec).to_radians().value;
            return units::MotorAngularVelocity(rad_per_sec);
        }
        if (motors.empty())
            return units::MotorAngularVelocity(0.0);

        double sum = 0.0;
        for (auto m : motors)
            sum += m->get_raw_velocity().rad_per_sec; // Extract the rad/s value

        return units::MotorAngularVelocity(sum / motors.size());
    }

    units::MotorPosition AdvancedMotorGroup::get_raw_position() const
    {
        if (using_encoder)
        {
            // Rotation sensor returns centidegrees
            double centidegrees = rotation_sensor->get_position();
            double degrees = centidegrees / 100.0;
            return units::MotorPosition(units::Degrees(degrees));
        }
        if (motors.empty())
            return units::MotorPosition(units::Radians(0.0));

        double sum_rad = 0.0;
        for (auto m : motors)
            sum_rad += m->get_raw_position().angle.value; // Access radians directly

        return units::MotorPosition(units::Radians(sum_rad / motors.size()));
    }

    units::Degrees AdvancedMotorGroup::get_position() const
    {
        return get_raw_position().angle.to_degrees();
    }

    units::RPM AdvancedMotorGroup::get_velocity() const
    {
        return units::RPM::from_rad_per_sec(get_raw_velocity().rad_per_sec);
    }

    void AdvancedMotorGroup::set_feedforward(bool enable)
    {
        use_feedforward = enable;
    }

    bool AdvancedMotorGroup::is_using_feedforward() const
    {
        return use_feedforward;
    }

    units::Current AdvancedMotorGroup::get_current_draw() const
    {
        double sum = 0.0;
        for (auto m : motors)
            sum += m->get_current_draw().amps;
        return units::Current::from_amps(sum);
    }

    units::Current AdvancedMotorGroup::get_average_current_draw() const
    {
        if (motors.empty())
            return units::Current::from_amps(0.0);
        return units::Current::from_amps(get_current_draw().amps / motors.size());
    }

    std::vector<units::Current> AdvancedMotorGroup::get_individual_current_draws() const
    {
        std::vector<units::Current> currents;
        currents.reserve(motors.size());
        for (auto m : motors)
            currents.push_back(m->get_current_draw());
        return currents;
    }

    void AdvancedMotorGroup::rotate_to(units::Degrees target, units::Time timeout,
                                       units::Voltage min_voltage, units::Voltage max_voltage)
    {
        reset_position();
        uint32_t start = pros::millis();
        const units::Degrees threshold(1.0);
        const double dt = 0.01;
        position_pid.reset();

        while ((pros::millis() - start) < timeout.to_millis_uint())
        {
            if (pros::Task::notify_take(true, 0) > 0)
            {
                break;
            }

            units::Degrees pos = get_position();
            units::MotorAngularVelocity vel = get_raw_velocity();

            if (std::fabs((target - pos).value) <= threshold.value &&
                std::fabs(vel.rad_per_sec) < 0.1)
                break;

            double err = (target - pos).value;
            double out = position_pid.compute(err, dt);

            units::Voltage output_voltage(out);
            output_voltage = units::Voltage(std::clamp(output_voltage.volts, min_voltage.volts, max_voltage.volts));
            move_voltage(output_voltage);
            pros::delay(10);
        }
        brake();
    }

    void AdvancedMotorGroup::rotate_to_task(units::Degrees target, units::Time timeout,
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

    void AdvancedMotorGroup::hold_velocity(units::RPM target_rpm, units::Time timeout,
                                           units::Voltage min_voltage, units::Voltage max_voltage)
    {
        uint32_t start = pros::millis();
        const double dt = 0.01;
        velocity_pid.reset();

        units::MotorAngularVelocity target_velocity = units::MotorAngularVelocity::from_rpm(target_rpm.value);

        while ((pros::millis() - start) < timeout.to_millis_uint())
        {
            if (pros::Task::notify_take(true, 0) > 0)
            {
                break;
            }

            units::MotorAngularVelocity vel = get_raw_velocity();

            double err = target_velocity.rad_per_sec - vel.rad_per_sec;
            double out = velocity_pid.compute(err, dt);

            if (use_feedforward)
            {
                double ff = group_config.kS * ((target_velocity.rad_per_sec > 0) - (target_velocity.rad_per_sec < 0));
                ff += group_config.kV * target_velocity.rad_per_sec;
                out += ff;
            }

            units::Voltage output_voltage(out);
            output_voltage = units::Voltage(std::clamp(output_voltage.volts, min_voltage.volts, max_voltage.volts));
            move_voltage(output_voltage);
            pros::delay(10);
        }
    }

    void AdvancedMotorGroup::hold_velocity_task(units::RPM target_rpm, units::Time timeout,
                                                units::Voltage min_voltage, units::Voltage max_voltage)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        // Signal existing task to stop
        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        current_task_ = pros::Task([this, target_rpm, timeout, min_voltage, max_voltage]()
                                   { this->hold_velocity(target_rpm, timeout, min_voltage, max_voltage); });
    }

    void AdvancedMotorGroup::move_velocity_continuous(units::MotorAngularVelocity target_velocity)
    {
        const double dt = 0.01; // Assumes 100Hz calls

        units::MotorAngularVelocity vel = get_raw_velocity();
        double err = target_velocity.rad_per_sec - vel.rad_per_sec;
        double out = velocity_pid.compute(err, dt);

        // Add feedforward if enabled
        if (use_feedforward)
        {
            double ff = group_config.kS * ((target_velocity.rad_per_sec > 0) - (target_velocity.rad_per_sec < 0));
            ff += group_config.kV * target_velocity.rad_per_sec;
            out += ff;
        }

        units::Voltage output_voltage(out);
        output_voltage = units::Voltage(std::clamp(output_voltage.volts, -12.0, 12.0));
        move_voltage(output_voltage);
    }

    void AdvancedMotorGroup::move_velocity_continuous(units::MotorAngularVelocity target_velocity,
                                                      double override_kS,
                                                      double override_kV)
    {
        const double dt = 0.01; // Assumes 100Hz calls

        units::MotorAngularVelocity vel = get_raw_velocity();
        double err = target_velocity.rad_per_sec - vel.rad_per_sec;
        double out = velocity_pid.compute(err, dt);

        // Use override constants
        if (use_feedforward)
        {
            double ff = override_kS * ((target_velocity.rad_per_sec > 0) - (target_velocity.rad_per_sec < 0));
            ff += override_kV * target_velocity.rad_per_sec;
            out += ff;
        }

        units::Voltage output_voltage(out);
        output_voltage = units::Voltage(std::clamp(output_voltage.volts, -12.0, 12.0));
        move_voltage(output_voltage);
    }

    void AdvancedMotorGroup::move_velocity_continuous_task(units::MotorAngularVelocity target_velocity)
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
        // Stop motor when task exits
        this->brake(); });
    }

    void AdvancedMotorGroup::stop_all_tasks()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        if (current_task_.has_value())
        {
            current_task_->notify();
            current_task_ = std::nullopt; // Clear the task
        }

        brake(); // Stop the motors
    }

} // namespace abclib::hardware