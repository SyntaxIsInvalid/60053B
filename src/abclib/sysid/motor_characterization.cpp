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
            owned_motors_.push_back(std::make_unique<pros::Motor>(port, gearset));

            MotorConfig motor_cfg{};
            motor_cfg.brake_mode = pros::E_MOTOR_BRAKE_COAST;
            motor_cfg.enable_voltage_compensation = config.enable_voltage_compensation;
            motor_cfg.compensation_nominal = config.compensation_nominal;
            motor_cfg.compensation_min_battery = config.compensation_min_battery;

            owned_advanced_motors_.push_back(
                std::make_unique<AdvancedMotor>(owned_motors_.back().get(), motor_cfg));

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

    units::AngularVelocity AdvancedMotorGroup::get_raw_velocity() const
    {
        if (using_encoder)
        {
            // Rotation sensor returns centidegrees/s, convert to rad/s
            double deg_per_sec = rotation_sensor->get_velocity() / 100.0;
            return units::AngularVelocity::from_deg_per_sec(deg_per_sec);
        }
        if (motors.empty())
            return units::AngularVelocity::from_rad_per_sec(0.0);

        double sum = 0.0;
        for (auto m : motors)
            sum += m->get_raw_velocity().to_rad_per_sec();

        return units::AngularVelocity::from_rad_per_sec(sum / motors.size());
    }

    units::Angle AdvancedMotorGroup::get_raw_position() const
    {
        if (using_encoder)
        {
            // Rotation sensor returns centidegrees
            double centidegrees = rotation_sensor->get_position();
            double degrees = centidegrees / 100.0;
            return units::Angle::from_degrees(degrees);
        }
        if (motors.empty())
            return units::Angle::from_radians(0.0);

        double sum_rad = 0.0;
        for (auto m : motors)
            sum_rad += m->get_raw_position().to_radians();

        return units::Angle::from_radians(sum_rad / motors.size());
    }

    units::Angle AdvancedMotorGroup::get_position() const
    {
        return get_raw_position();
    }

    units::AngularVelocity AdvancedMotorGroup::get_velocity() const
    {
        return get_raw_velocity();
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
            sum += m->get_current_draw().to_amps();
        return units::Current::from_amps(sum);
    }

    units::Current AdvancedMotorGroup::get_average_current_draw() const
    {
        if (motors.empty())
            return units::Current::from_amps(0.0);
        return units::Current::from_amps(get_current_draw().to_amps() / motors.size());
    }

    std::vector<units::Current> AdvancedMotorGroup::get_individual_current_draws() const
    {
        std::vector<units::Current> currents;
        currents.reserve(motors.size());
        for (auto m : motors)
            currents.push_back(m->get_current_draw());
        return currents;
    }

    void AdvancedMotorGroup::rotate_to(units::Angle target, units::Time timeout,
                                       units::Voltage min_voltage, units::Voltage max_voltage)
    {
        reset_position();
        uint32_t start = pros::millis();
        const double threshold = 1.0; // degrees
        const double dt = 0.01;
        position_pid.reset();

        while ((pros::millis() - start) < timeout.to_milliseconds())
        {
            if (pros::Task::notify_take(true, 0) > 0)
            {
                break;
            }

            units::Angle pos = get_position();
            units::AngularVelocity vel = get_raw_velocity();

            double pos_deg = pos.to_degrees();
            double target_deg = target.to_degrees();

            if (std::fabs(target_deg - pos_deg) <= threshold &&
                std::fabs(vel.to_rad_per_sec()) < 0.1)
                break;

            double err = target_deg - pos_deg;
            double out = position_pid.compute(err, dt);

            units::Voltage output_voltage = units::Voltage::from_volts(
                std::clamp(out, min_voltage.to_volts(), max_voltage.to_volts()));
            move_voltage(output_voltage);
            pros::delay(10);
        }
        brake();
    }

    void AdvancedMotorGroup::rotate_to_task(units::Angle target, units::Time timeout,
                                            units::Voltage min_voltage, units::Voltage max_voltage)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        current_task_ = pros::Task([this, target, timeout, min_voltage, max_voltage]()
                                   { this->rotate_to(target, timeout, min_voltage, max_voltage); });
    }

    void AdvancedMotorGroup::hold_velocity(units::AngularVelocity target_velocity, units::Time timeout,
                                           units::Voltage min_voltage, units::Voltage max_voltage)
    {
        uint32_t start = pros::millis();
        const double dt = 0.01;
        velocity_pid.reset();

        while ((pros::millis() - start) < timeout.to_milliseconds())
        {
            if (pros::Task::notify_take(true, 0) > 0)
            {
                break;
            }

            units::AngularVelocity vel = get_raw_velocity();

            double target_rad_s = target_velocity.to_rad_per_sec();
            double vel_rad_s = vel.to_rad_per_sec();
            double err = target_rad_s - vel_rad_s;
            double out = velocity_pid.compute(err, dt);

            if (use_feedforward)
            {
                double ff = group_config.kS * ((target_rad_s > 0) - (target_rad_s < 0));
                ff += group_config.kV * target_rad_s;
                out += ff;
            }

            units::Voltage output_voltage = units::Voltage::from_volts(
                std::clamp(out, min_voltage.to_volts(), max_voltage.to_volts()));
            move_voltage(output_voltage);
            pros::delay(10);
        }
    }

    void AdvancedMotorGroup::hold_velocity_task(units::AngularVelocity target_velocity, units::Time timeout,
                                                units::Voltage min_voltage, units::Voltage max_voltage)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        current_task_ = pros::Task([this, target_velocity, timeout, min_voltage, max_voltage]()
                                   { this->hold_velocity(target_velocity, timeout, min_voltage, max_voltage); });
    }

    void AdvancedMotorGroup::move_velocity_continuous(units::AngularVelocity target_velocity,
                                                      double target_acceleration)
    {
        const double dt = 0.01;

        units::AngularVelocity vel = get_raw_velocity();
        double target_rad_s = target_velocity.to_rad_per_sec();
        double vel_rad_s = vel.to_rad_per_sec();
        double err = target_rad_s - vel_rad_s;
        double out = velocity_pid.compute(err, dt);

        if (use_feedforward)
        {
            double ff = group_config.kS * ((target_rad_s > 0) - (target_rad_s < 0));
            ff += group_config.kV * target_rad_s;
            ff += group_config.kA * target_acceleration;
            out += ff;
        }

        units::Voltage output_voltage = units::Voltage::from_volts(std::clamp(out, -12.0, 12.0));
        move_voltage(output_voltage);
    }

    void AdvancedMotorGroup::move_velocity_continuous(units::AngularVelocity target_velocity, 
                                                      double target_acceleration,
                                                      double override_kS,
                                                      double override_kV,
                                                      double override_kA)
    {
        const double dt = 0.01;

        units::AngularVelocity vel = get_raw_velocity();
        double target_rad_s = target_velocity.to_rad_per_sec();
        double vel_rad_s = vel.to_rad_per_sec();
        double err = target_rad_s - vel_rad_s;
        double out = velocity_pid.compute(err, dt);

        if (use_feedforward)
        {
            double ff = override_kS * ((target_rad_s > 0) - (target_rad_s < 0));
            ff += override_kV * target_rad_s;
            ff += override_kA * target_acceleration;
            out += ff;
        }

        units::Voltage output_voltage = units::Voltage::from_volts(std::clamp(out, -12.0, 12.0));
        move_voltage(output_voltage);
    }

    void AdvancedMotorGroup::move_velocity_continuous_task(units::AngularVelocity target_velocity)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        current_task_ = pros::Task([this, target_velocity]()
                                   {
            while (pros::Task::notify_take(true, 0) == 0) {
                this->move_velocity_continuous(target_velocity, 0.0);
                pros::delay(10);
            }
            this->brake(); });
    }

    void AdvancedMotorGroup::stop_all_tasks()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        if (current_task_.has_value())
        {
            current_task_->notify();
            current_task_ = std::nullopt;
        }

        brake();
    }

    void AdvancedMotorGroup::move_velocity_pros(units::AngularVelocity target_velocity)
    {
        for (auto m : motors)
        {
            m->move_velocity_pros(target_velocity);
        }
    }

    void AdvancedMotorGroup::move_velocity_pros_task(units::AngularVelocity target_velocity)
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        if (current_task_.has_value())
        {
            current_task_->notify();
        }

        current_task_ = pros::Task([this, target_velocity]()
                                   {
            while (pros::Task::notify_take(true, 0) == 0) {
                this->move_velocity_pros(target_velocity);
                pros::delay(10);
            }
            this->brake(); });
    }

} // namespace abclib::hardware