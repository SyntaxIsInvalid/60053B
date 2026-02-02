#pragma once

#include <vector>
#include "advanced_motor.hpp"
#include "abclib/control/pid/pid.hpp"
#include "abclib/units/units.hpp"

namespace abclib::hardware
{
    struct motor_group_config
    {
        double kS = 0;
        double kV = 0;
        double kA = 0;

        double kPs = 0;
        double kIs = 0;
        double kDs = 0;
        double max_integral_position = 100;

        double kPv = 0;
        double kIv = 0;
        double kDv = 0;
        double max_integral_velocity = 100;

        const double group_gearing = 1;
        pros::Rotation *rotation = nullptr;

        bool enable_voltage_compensation = false;
        units::Voltage compensation_nominal = units::Voltage::from_volts(12.0);
        units::Voltage compensation_min_battery = units::Voltage::from_volts(11.5);
    };

    class AdvancedMotorGroup
    {
    private:
        std::vector<AdvancedMotor *> motors;
        std::vector<std::unique_ptr<pros::Motor>> owned_motors_;
        std::vector<std::unique_ptr<AdvancedMotor>> owned_advanced_motors_;
        pros::Rotation *rotation_sensor;
        control::PID position_pid;
        control::PID velocity_pid;

        motor_group_config group_config;
        double ticks;
        double further_gearing;
        bool use_feedforward;
        bool using_encoder;

        std::optional<pros::Task> current_task_;
        mutable pros::Mutex task_mutex_;

    public:
        AdvancedMotorGroup(
            const std::vector<int8_t> &ports,
            pros::MotorGearset gearset,
            const motor_group_config &config = {},
            bool enable_feedforward = true);

        AdvancedMotorGroup(
            const std::vector<AdvancedMotor *> &motors_list,
            const motor_group_config &config,
            bool enable_feedforward = true);
        ~AdvancedMotorGroup();

        double get_velocity_p_term() const { return velocity_pid.get_p_term(); }
        double get_velocity_i_term() const { return velocity_pid.get_i_term(); }
        double get_velocity_d_term() const { return velocity_pid.get_d_term(); }

        void reset_position();
        void move_voltage(units::Voltage voltage);
        void set_brake_mode(pros::motor_brake_mode_e_t brake_mode);
        void brake();
        double get_ticks();
        units::AngularVelocity get_velocity() const;
        units::Angle get_position() const;
        void set_feedforward(bool enable);
        bool is_using_feedforward() const;
        units::Current get_current_draw() const;
        units::Current get_average_current_draw() const;
        std::vector<units::Current> get_individual_current_draws() const;

        void rotate_to(units::Angle target,
                       units::Time timeout = units::Time::from_seconds(3.0),
                       units::Voltage min_voltage = units::Voltage::from_volts(-12.0),
                       units::Voltage max_voltage = units::Voltage::from_volts(12.0));
        void rotate_to_task(units::Angle target,
                            units::Time timeout = units::Time::from_seconds(3.0),
                            units::Voltage min_voltage = units::Voltage::from_volts(-12.0),
                            units::Voltage max_voltage = units::Voltage::from_volts(12.0));
        void hold_velocity(units::AngularVelocity target_velocity,
                           units::Time timeout = units::Time::from_seconds(3.0),
                           units::Voltage min_voltage = units::Voltage::from_volts(-12.0),
                           units::Voltage max_voltage = units::Voltage::from_volts(12.0));
        void hold_velocity_task(units::AngularVelocity target_velocity,
                                units::Time timeout = units::Time::from_seconds(3.0),
                                units::Voltage min_voltage = units::Voltage::from_volts(-12.0),
                                units::Voltage max_voltage = units::Voltage::from_volts(12.0));
        void move_velocity_continuous(units::AngularVelocity target_velocity,
                                      double target_acceleration);

        void move_velocity_continuous(units::AngularVelocity target_velocity,
                                      double target_acceleration,
                                      double override_kS,
                                      double override_kV,
                                      double override_kA);
        void move_velocity_continuous_task(units::AngularVelocity target_velocity);

        units::AngularVelocity get_raw_velocity() const;
        units::Angle get_raw_position() const;
        motor_group_config get_config() const { return group_config; }
        void stop_all_tasks();
        void move_velocity_pros(units::AngularVelocity target_velocity);
        void move_velocity_pros_task(units::AngularVelocity target_velocity);
        void set_config(const motor_group_config &config)
        {
            // Update config members individually (skip rotation sensor)
            group_config.kS = config.kS;
            group_config.kV = config.kV;
            group_config.kA = config.kA;

            group_config.kPs = config.kPs;
            group_config.kIs = config.kIs;
            group_config.kDs = config.kDs;
            group_config.max_integral_position = config.max_integral_position;

            group_config.kPv = config.kPv;
            group_config.kIv = config.kIv;
            group_config.kDv = config.kDv;
            group_config.max_integral_velocity = config.max_integral_velocity;

            group_config.enable_voltage_compensation = config.enable_voltage_compensation;
            group_config.compensation_nominal = config.compensation_nominal;
            group_config.compensation_min_battery = config.compensation_min_battery;

            // Update PID controllers with new constants
            position_pid.set_constants({config.kPs,
                                        config.kIs,
                                        config.kDs,
                                        config.max_integral_position});

            velocity_pid.set_constants({config.kPv,
                                        config.kIv,
                                        config.kDv,
                                        config.max_integral_velocity});

            // Update feedforward flag
            use_feedforward = (config.kS != 0 || config.kV != 0 || config.kA != 0);
        }
    };
}