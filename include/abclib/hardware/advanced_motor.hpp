#pragma once

#include "api.h"
#include <cmath>
#include <memory>
#include <algorithm>
#include "abclib/control/pid/pid.hpp"
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
        pros::motor_brake_mode_e_t brake_mode = pros::E_MOTOR_BRAKE_COAST;

        bool enable_voltage_compensation = false;
        units::Voltage compensation_nominal = units::Voltage::from_volts(12.0);
        units::Voltage compensation_min_battery = units::Voltage::from_volts(11.5);
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

        void rotate_to(units::Angle target,
                       units::Time timeout = units::Time::from_seconds(3.0),
                       units::Voltage min_voltage = units::Voltage::from_volts(-12.0),
                       units::Voltage max_voltage = units::Voltage::from_volts(12.0));

        void rotate_to_task(units::Angle target,
                            units::Time timeout = units::Time::from_seconds(3.0),
                            units::Voltage min_voltage = units::Voltage::from_volts(-12.0),
                            units::Voltage max_voltage = units::Voltage::from_volts(12.0));

        void hold_velocity(units::AngularVelocity target_vel,
                           units::Time timeout = units::Time::from_seconds(3.0),
                           units::Voltage min_voltage = units::Voltage::from_volts(-12.0),
                           units::Voltage max_voltage = units::Voltage::from_volts(12.0));

        void hold_velocity_task(units::AngularVelocity target_vel,
                                units::Time timeout = units::Time::from_seconds(3.0),
                                units::Voltage min_voltage = units::Voltage::from_volts(-12.0),
                                units::Voltage max_voltage = units::Voltage::from_volts(12.0));

        void move_velocity_continuous(units::AngularVelocity target_velocity);
        void move_velocity_continuous(units::AngularVelocity target_velocity,
                                      double override_kS,
                                      double override_kV);
        void move_velocity_continuous_task(units::AngularVelocity target_velocity);

        void set_feedforward(bool enable);
        bool is_using_feedforward() const;
        units::AngularVelocity get_velocity() const;
        units::Angle get_position() const;
        std::optional<pros::Task> current_task_;
        mutable pros::Mutex task_mutex_;

        units::AngularVelocity get_raw_velocity() const;
        units::Angle get_raw_position() const;

        void move_velocity_pros(units::AngularVelocity target_velocity);
        void move_velocity_pros_task(units::AngularVelocity target_velocity);

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