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
    inline const std::vector<int8_t> TOP_INTAKE_PORTS = {-9};
    inline const std::vector<int8_t> BOTTOM_INTAKE_PORTS = {10};

    // Sensor ports
    constexpr int8_t IMU_PORT = 18;
    constexpr int8_t Y_ROTATION_PORT = -17;

    // Pneumatic ports
    constexpr char MATCH_LOAD_RAMP_PORT = 'G';
    constexpr char INTAKE_LIFT_PORT = 'H';
    // constexpr char INTAKE_DOOR = ''

    // Physical dimensions
    constexpr double WHEEL_DIAMETER_INCHES = 2.75;
    constexpr double TRACK_WIDTH_INCHES = 14.0;
    constexpr double Y_TRACKER_WHEEL_DIAMETER = 2.0;
    constexpr double Y_TRACKER_OFFSET_INCHES = 0.0;

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

    // Chassis config values (untuned)
    constexpr double TURN_IN_PLACE_KS = 0;
    constexpr double TURN_IN_PLACE_KV = 0;
    constexpr double TURN_IN_PLACE_KA = 0;
}