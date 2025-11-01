#pragma once

#include <vector>
#include "advanced_motor.hpp"
#include "abclib/control/pid.hpp"
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
        units::RPM get_velocity() const;
        units::Degrees get_position() const;
        void set_feedforward(bool enable);
        bool is_using_feedforward() const;
        units::Current get_current_draw() const;
        units::Current get_average_current_draw() const;
        std::vector<units::Current> get_individual_current_draws() const;

        void rotate_to(units::Degrees target,
                       units::Time timeout = units::DEFAULT_TIMEOUT,
                       units::Voltage min_voltage = units::DEFAULT_MIN_VOLTAGE,
                       units::Voltage max_voltage = units::DEFAULT_MAX_VOLTAGE);
        void rotate_to_task(units::Degrees target,
                            units::Time timeout = units::DEFAULT_TIMEOUT,
                            units::Voltage min_voltage = units::DEFAULT_MIN_VOLTAGE,
                            units::Voltage max_voltage = units::DEFAULT_MAX_VOLTAGE);
        void hold_velocity(units::RPM target_rpm,
                           units::Time timeout = units::DEFAULT_TIMEOUT,
                           units::Voltage min_voltage = units::DEFAULT_MIN_VOLTAGE,
                           units::Voltage max_voltage = units::DEFAULT_MAX_VOLTAGE);
        void hold_velocity_task(units::RPM target_rpm,
                                units::Time timeout = units::DEFAULT_TIMEOUT,
                                units::Voltage min_voltage = units::DEFAULT_MIN_VOLTAGE,
                                units::Voltage max_voltage = units::DEFAULT_MAX_VOLTAGE);
        void move_velocity_continuous(units::MotorAngularVelocity target_velocity);
        void move_velocity_continuous(units::MotorAngularVelocity target_velocity,
                                      double override_kS,
                                      double override_kV);
        void move_velocity_continuous_task(units::MotorAngularVelocity target_velocity);

        units::MotorAngularVelocity get_raw_velocity() const;
        units::MotorPosition get_raw_position() const;
        motor_group_config get_config() const { return group_config; }
        void stop_all_tasks();
    };
}