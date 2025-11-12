#pragma once
#include "config.hpp"

namespace abclib::config {

    inline RobotConfig get_competition_robot_defaults() {
        RobotConfig config;
        
        config.robot_name = "competition_robot";
        config.version = "1.0";
        config.source = ConfigSource::HARDCODED;
        
        config.left_motor = {
            .kS = 0.0,
            .kV = 0.0,
            .kA = 0.0,
            .kPv = 0.0,
            .kIv = 0.0,
            .kDv = 0.0,
            .enable_voltage_compensation = true,
            .compensation_nominal = 12.0,
            .compensation_min_battery = 11.5
        };
        
        config.right_motor = config.left_motor;
        
        config.lateral_pid = { .kP = 0.45, .kI = 0.0, .kD = 0.0 };
        config.angular_pid = { .kP = 4.7, .kI = 0.0, .kD = 0.0 };
        config.profiled_turn_pid = { .kP = 0.0, .kI = 0.0, .kD = 0.0 };
        
        config.turn_in_place_ff = {
            .kS = 0.919850,
            .kV = 0.169750,
            .kA = 0.0
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