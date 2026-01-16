#pragma once
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/tracking_wheel.hpp"
#include "abclib/units/units.hpp"
#include "api.h"
#include "abclib/control/ramsete.hpp"
#include "abclib/estimation/estimator_config.hpp"
namespace abclib::robot_config
{

    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-11, -13, 12};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {19, 20, -16};
    inline const std::vector<int8_t> TOP_INTAKE_PORTS = {-1};
    inline const std::vector<int8_t> BOTTOM_INTAKE_PORTS = {10};

    // Intake voltages - now using units::Voltage
    inline const units::Voltage TOP_INTAKE_VOLTAGE = units::Voltage::from_volts(12.0);
    inline const units::Voltage TOP_OUTTAKE_VOLTAGE = units::Voltage::from_volts(-12.0);
    inline const units::Voltage BOTTOM_INTAKE_VOLTAGE = units::Voltage::from_volts(12.0);
    inline const units::Voltage BOTTOM_OUTTAKE_VOLTAGE = units::Voltage::from_volts(-12.0);

    // Sensor ports
    constexpr int8_t IMU_PORT = 17;
    constexpr int8_t Y_ROTATION_PORT = -18;
    constexpr int8_t FRONT_DISTANCE_SENSOR_PORT = 15;
    constexpr int8_t BACK_DISTANCE_SENSOR_PORT = 14;

    // Pneumatic ports
    constexpr char MATCH_LOAD_RAMP_PORT = 'A';
    constexpr char HOOD_PORT = 'F';
    constexpr char WING_PORT = 'E';

    // Physical dimensions - now using units::Length
    inline const units::Length WHEEL_DIAMETER = units::Length::from_inches(3.25);
    inline const units::Length TRACK_WIDTH = units::Length::from_inches(14.0);
    inline const units::Length Y_TRACKER_WHEEL_DIAMETER = units::Length::from_inches(2.0);
    inline const units::Length Y_TRACKER_OFFSET = units::Length::from_inches(0.0);

    inline std::pair<pros::Distance *, pros::Distance *> get_distance_sensors()
    {
        static pros::Distance front_sensor(FRONT_DISTANCE_SENSOR_PORT);
        static pros::Distance back_sensor(BACK_DISTANCE_SENSOR_PORT);

        return {&front_sensor, &back_sensor};
    }

    // Motor configurations (untuned)
    inline hardware::motor_group_config get_left_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 1.148090,
            .kV = 0.165643,
            .kA = 0,
            .kPv = 0,
            .kIv = 0,
            .kDv = 0,
            .enable_voltage_compensation = true};
    }

    inline hardware::motor_group_config get_right_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 1.148090,
            .kV = 0.165643,
            .kA = 0,
            .kPv = 0,
            .kIv = 0,
            .kDv = 0,
            .enable_voltage_compensation = true};
    }

    // PID constants
    inline control::PIDConstants get_lateral_pid()
    {
        return control::PIDConstants(0.22, 0, 0);
    }

    inline control::PIDConstants get_angular_pid()
    {
        // return control::PIDConstants(4.7, 0, 0);
        return control::PIDConstants(4.7, 0, 0);
    }

    // profiled turn pid constants
    inline control::PIDConstants get_profiled_turn_pid()
    {
        return control::PIDConstants(6.2, 0.0, 0.0); // Tune these values
    }

    // profiled lateral pid constants
    inline control::PIDConstants get_profiled_lateral_pid()
    {
        return control::PIDConstants(0.0, 0.0, 0.0); // Tune these values
    }

    inline control::RamseteConstants get_ramsete_config()
    {
        return control::RamseteConstants{2.0, 0.7}; // b, zeta
    }

    // Chassis config values (untuned) - these remain as raw doubles since they're feedforward gains
    constexpr double TURN_IN_PLACE_KS = 1.227080;
    constexpr double TURN_IN_PLACE_KV = 0.160866;
    constexpr double TURN_IN_PLACE_KA = 0;

    constexpr double LATERAL_KS = 0;
    constexpr double LATERAL_KV = 0;
    constexpr double LATERAL_KA = 0;

    inline estimation::EstimatorConfig get_estimator_config()
    {
        estimation::EstimatorConfig config;
        config.type = estimation::FilterType::GEOMETRIC; // or GEOMETRIC/EKF
        config.mode = estimation::FilterMode::FULL;
        config.field_config = field::FieldConfig::custom(
            units::Length::from_inches(24), // width
            units::Length::from_inches(48)  // height
        );
        return config;
    }
}