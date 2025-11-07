#pragma once
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/tracking_wheel.hpp"
#include "abclib/units/units.hpp"
#include "api.h"

namespace abclib::robot_config
{

    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-12, -11, -19};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {14, 15, 20};
    inline const std::vector<int8_t> TOP_INTAKE_PORTS = {9};
    inline const std::vector<int8_t> BOTTOM_INTAKE_PORTS = {-10};

    // Intake voltages - now using units::Voltage
    inline const units::Voltage TOP_INTAKE_VOLTAGE = units::Voltage::from_volts(8.0);
    inline const units::Voltage TOP_OUTTAKE_VOLTAGE = units::Voltage::from_volts(-12.0);
    inline const units::Voltage BOTTOM_INTAKE_VOLTAGE = units::Voltage::from_volts(12.0);
    inline const units::Voltage BOTTOM_OUTTAKE_VOLTAGE = units::Voltage::from_volts(-12.0);

    // Sensor ports
    constexpr int8_t IMU_PORT = 18;
    constexpr int8_t Y_ROTATION_PORT = -17;

    // Pneumatic ports
    constexpr char MATCH_LOAD_RAMP_PORT = 'G';
    constexpr char INTAKE_LIFT_PORT = 'H';

    // Physical dimensions - now using units::Distance
    inline const units::Distance WHEEL_DIAMETER = units::Distance::from_inches(2.75);
    inline const units::Distance TRACK_WIDTH = units::Distance::from_inches(14.0);
    inline const units::Distance Y_TRACKER_WHEEL_DIAMETER = units::Distance::from_inches(2.0);
    inline const units::Distance Y_TRACKER_OFFSET = units::Distance::from_inches(0.0);

    // Motor configurations (untuned)
    inline hardware::motor_group_config get_left_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 0,
            .kV = 0,
            .kA = 0,
            .kPv = 0,
            .kIv = 0,
            .kDv = 0,
            .enable_voltage_compensation = true};
    }

    inline hardware::motor_group_config get_right_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 0,
            .kV = 0,
            .kA = 0,
            .kPv = 0,
            .kIv = 0,
            .kDv = 0,
            .enable_voltage_compensation = true};
    }

    // PID constants
    inline control::PIDConstants get_lateral_pid()
    {
        return control::PIDConstants(0.45, 0, 0);
    }

    inline control::PIDConstants get_angular_pid()
    {
        return control::PIDConstants(4.7, 0, 0);
    }

    // Chassis config values (untuned) - these remain as raw doubles since they're feedforward gains
    constexpr double TURN_IN_PLACE_KS = 0;
    constexpr double TURN_IN_PLACE_KV = 0;
    constexpr double TURN_IN_PLACE_KA = 0;
}