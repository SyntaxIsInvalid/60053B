#pragma once
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/motor_tracking_wheel.hpp"
#include "abclib/units/units.hpp"
#include "api.h"

namespace abclib::robot_config {
        
    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-10, 4};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {6, -11};
    
    // Sensor ports
    constexpr int8_t IMU_PORT = 9;
    
    // Physical dimensions
    constexpr double WHEEL_DIAMETER_INCHES = 3.25;
    constexpr double TRACK_WIDTH_INCHES = 14.0;
    constexpr double Y_TRACKER_OFFSET_INCHES = 7.0;
    
    // Motor configurations
    inline hardware::motor_group_config get_left_motor_config() {
        return hardware::motor_group_config{
            .kS = 0.919850,
            .kV = 0.1594417,
            .kA = 0.012848,
            .kPv = 0.18,
            .kIv = 0.25,
            .kDv = 0.00,
            .enable_voltage_compensation = true
        };
    }
    
    inline hardware::motor_group_config get_right_motor_config() {
        return hardware::motor_group_config{
            .kS = 0.919850,
            .kV = 0.1594417,
            .kA = 0.012848,
            .kPv = 0.18,
            .kIv = 0.25,
            .kDv = 0.00,
            .enable_voltage_compensation = true
        };
    }
    
    // PID constants
    inline control::PIDConstants get_lateral_pid() {
        return control::PIDConstants(0.5, 0, 0);
    }
    
    inline control::PIDConstants get_angular_pid() {
        return control::PIDConstants(5, 0, 0);
    }
    
    // Chassis config values
    constexpr double TURN_IN_PLACE_KS = 1.278592;
    constexpr double TURN_IN_PLACE_KV = 0.170242;
    constexpr double TURN_IN_PLACE_KA = 0.012877;
}