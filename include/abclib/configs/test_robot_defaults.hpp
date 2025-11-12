#pragma once
#include "config.hpp"

namespace abclib::config {

    inline RobotConfig get_test_robot_defaults() {
        RobotConfig config;
        
        config.robot_name = "test_robot";
        config.version = "1.0";
        config.source = ConfigSource::HARDCODED;
        
        // Left motor
        config.left_motor = {
            .kS = 0.919850,
            .kV = 0.1594417,
            .kA = 0.012848,
            .kPv = 0.18,
            .kIv = 0.25,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = 12.0,
            .compensation_min_battery = 11.5
        };
        
        config.right_motor = config.left_motor;
        
        config.lateral_pid = { .kP = 0.5, .kI = 0.0, .kD = 0.0 };
        config.angular_pid = { .kP = 5.0, .kI = 0.0, .kD = 0.0 };
        config.profiled_turn_pid = { .kP = 25.0, .kI = 0.0, .kD = 0.0 };
        
        config.turn_in_place_ff = {
            .kS = 1.278592,
            .kV = 0.170242,
            .kA = 0.012877
        };
        
        config.settlement = {
            .angular_threshold_deg = 1.0,
            .position_threshold_inches = 0.5,
            .angular_velocity_threshold = 0.1,
            .linear_velocity_threshold = 0.15,
            .settle_count_required = 3
        };
        
        return config;
    }

} // namespace abclib::config