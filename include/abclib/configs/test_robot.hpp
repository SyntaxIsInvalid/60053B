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
    
    // Physical dimensions (using typed units)
    inline constexpr units::Distance WHEEL_DIAMETER = units::Distance::from_inches(3.25);
    inline constexpr units::Distance TRACK_WIDTH = units::Distance::from_inches(14.0);
    inline constexpr units::Distance Y_TRACKER_OFFSET = units::Distance::from_inches(7.0);
    
    // Motor configurations
    inline hardware::motor_group_config get_left_motor_config() {
        return hardware::motor_group_config{
            .kS = 0.919850,
            .kV = 0.1594417,
            .kA = 0.012848,
            .kPv = 0.18,
            .kIv = 0.25,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)
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
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)
        };
    }
    
    // PID constants
    inline control::PIDConstants get_lateral_pid() {
        return control::PIDConstants(0.5, 0, 0);
    }
    
    inline control::PIDConstants get_angular_pid() {
        return control::PIDConstants(5, 0, 0);
    }
    
    // Turn-in-place feedforward constants (for turn_to_heading_profiled)
    constexpr double TURN_IN_PLACE_KS = 1.278592;
    constexpr double TURN_IN_PLACE_KV = 0.170242;
    constexpr double TURN_IN_PLACE_KA = 0.012877;

    constexpr double LATERAL_KS = 0.919850;
    constexpr double LATERAL_KV = 0.1594417;
    constexpr double LATERAL_KA = 0.012848;
    
    // profiled turn pid constants
    inline control::PIDConstants get_profiled_turn_pid() {
        return control::PIDConstants(25, 0.0, 0.0);  // Tune these values
    }

    // profiled lateral pid constants
    inline control::PIDConstants get_profiled_lateral_pid() {
        return control::PIDConstants(0.0, 0.0, 0.0);  // Tune these values
    }

    // Ramsete controller constants (for path following)
}