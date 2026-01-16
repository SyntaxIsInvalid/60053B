#pragma once
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/motor_tracking_wheel.hpp"
#include "abclib/units/units.hpp"
#include "api.h"
#include "abclib/control/ramsete.hpp"
#include "abclib/estimation/estimator_config.hpp"
namespace abclib::robot_config
{

    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-10, 4};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {6, -11};

    // Sensor ports
    constexpr int8_t IMU_PORT = 9;
    constexpr int8_t FRONT_DISTANCE_SENSOR_PORT = 12; // YOUR ACTUAL PORT
    constexpr int8_t BACK_DISTANCE_SENSOR_PORT = 1;   // YOUR ACTUAL PORT

    // Physical dimensions (using typed units)
    inline constexpr units::Length WHEEL_DIAMETER = units::Length::from_inches(3.25);
    inline constexpr units::Length TRACK_WIDTH = units::Length::from_inches(14.0);
    inline constexpr units::Length Y_TRACKER_OFFSET = units::Length::from_inches(7.0);

    inline std::pair<pros::Distance *, pros::Distance *> get_distance_sensors()
    {
        static pros::Distance front_sensor(FRONT_DISTANCE_SENSOR_PORT);
        static pros::Distance back_sensor(BACK_DISTANCE_SENSOR_PORT);

        return {&front_sensor, &back_sensor};
    }

    // Motor configurations
    inline hardware::motor_group_config get_left_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 0.535278,
            .kV = 0.158462,
            .kA = 0.012848,
            .kPv = 0.0,
            .kIv = 0.25,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)};
    }

    inline hardware::motor_group_config get_right_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 0.535278,
            .kV = 0.158462,
            .kA = 0.012848,
            .kPv = 0.0,
            .kIv = 0.0,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)};
    }

    // PID constants
    inline control::PIDConstants get_lateral_pid()
    {
        return control::PIDConstants(0.6, 5, 0);
    }

    inline control::PIDConstants get_angular_pid()
    {
        return control::PIDConstants(5, 50, 0);
    }

    // Turn-in-place feedforward constants (for turn_to_heading_profiled)
    constexpr double TURN_IN_PLACE_KS = 1.278592;
    constexpr double TURN_IN_PLACE_KV = 0.170242;
    constexpr double TURN_IN_PLACE_KA = 0.012877;

    constexpr double LATERAL_KS = 0.535278;
    constexpr double LATERAL_KV = 0.158462;
    constexpr double LATERAL_KA = 0.012848;

    // profiled turn pid constants
    inline control::PIDConstants get_profiled_turn_pid()
    {
        return control::PIDConstants(25, 0.0, 0.0); // Tune these values
    }

    // profiled lateral pid constants
    inline control::PIDConstants get_profiled_lateral_pid()
    {
        return control::PIDConstants(2.0, 0.0, 0.0); // Tune these values
    }

    // Ramsete controller constants (for path following)
    inline control::RamseteConstants get_ramsete_config()
    {
        return control::RamseteConstants{2.0, 0.7}; // b, zeta
    }

    inline estimation::EstimatorConfig get_estimator_config()
    {
        estimation::EstimatorConfig config;
        config.type = estimation::FilterType::GEOMETRIC; // or GEOMETRIC/EKF
        config.mode = estimation::FilterMode::PREDICTION_ONLY;
        config.field_config = field::FieldConfig::custom(
            units::Length::from_inches(24), // width 
            units::Length::from_inches(48) // height 
        );
        return config;
    }

}