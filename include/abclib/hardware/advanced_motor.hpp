#pragma once

#include "api.h"
#include <cmath>
#include <memory>
#include <algorithm>
#include "abclib/control/pid.hpp"
#include "abclib/units/units.hpp"
#include <optional>

namespace abclib::hardware
{
    struct MotorConfig
    {
        double kS = 0.0;
        double kV = 0.0;
        double kA = 0.0;

        double kPs = 0.0;
        double kIs = 0.0;
        double kDs = 0.0;
        double max_integral_position = 100;

        double kPv = 0.0;
        double kIv = 0.0;
        double kDv = 0.0;
        double max_integral_velocity = 100;

        pros::Rotation *rotation = nullptr;
        pros::motor_brake_mode_e_t brake_mode = pros::E_MOTOR_BRAKE_BRAKE;
    };

    class AdvancedMotor
    {
    public:
        AdvancedMotor(int8_t port,
                      pros::MotorGearset gearset,
                      const MotorConfig &motor_config = {},
                      double gearing = 1.0,
                      bool enable_feedforward = true);

        AdvancedMotor(pros::Motor *base_motor,
                      const MotorConfig &motor_config,
                      double gearing = 1.0,
                      bool enable_feedforward = true);
        ~AdvancedMotor();

        static double find_ticks(pros::Motor *motor, double further_gearing);

        void reset_position();
        double get_ticks() const;
        units::Current get_current_draw() const;
        void move_voltage(units::Voltage voltage);
        void brake();
        void set_brake_mode(pros::motor_brake_mode_e_t brake_mode);

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

        void set_feedforward(bool enable);
        bool is_using_feedforward() const;
        units::RPM get_velocity() const;
        units::Degrees get_position() const;
        std::optional<pros::Task> current_task_;
        mutable pros::Mutex task_mutex_;

        units::MotorAngularVelocity get_raw_velocity() const;
        units::MotorPosition get_raw_position() const;

    private:
        pros::Motor *motor_;
        std::optional<pros::Motor> owned_motor_;
        pros::Rotation *rotation_sensor_;
        control::PID position_pid_;
        control::PID velocity_pid_;
        MotorConfig config_;
        double ticks_;
        double gearing_;
        bool using_encoder_;
        bool use_feedforward_;
    };
}