#pragma once
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/motor_tracking_wheel.hpp"
#include "abclib/units/units.hpp"
#include "abclib/configs/config_loader.hpp"
#include "api.h"

namespace abclib::robot_config {
        
    // ========== HARDWARE CONSTANTS (Never change) ==========
    
    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-10, 4};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {6, -11};
    
    // Sensor ports
    constexpr int8_t IMU_PORT = 9;
    
    // Physical dimensions (measured once, never change)
    inline constexpr units::Distance WHEEL_DIAMETER = 
        units::Distance::from_inches(3.25);
    inline constexpr units::Distance TRACK_WIDTH = 
        units::Distance::from_inches(14.0);
    inline constexpr units::Distance Y_TRACKER_OFFSET = 
        units::Distance::from_inches(7.0);
    
    
    // ========== TUNABLE VALUES (from config system) ==========
    
    // Get the loaded config (tries SD card, falls back to hardcoded)
    inline config::RobotConfig& get_tuning_config() {
        static config::RobotConfig config = 
            config::ConfigLoader::load("test_robot");
        return config;
    }
    
    // Motor configurations
    inline hardware::motor_group_config get_left_motor_config() {
        return get_tuning_config().left_motor.to_motor_group_config();
    }
    
    inline hardware::motor_group_config get_right_motor_config() {
        return get_tuning_config().right_motor.to_motor_group_config();
    }
    
    // PID constants
    inline control::PIDConstants get_lateral_pid() {
        return get_tuning_config().lateral_pid.to_pid_constants();
    }
    
    inline control::PIDConstants get_angular_pid() {
        return get_tuning_config().angular_pid.to_pid_constants();
    }
    
    inline control::PIDConstants get_profiled_turn_pid() {
        return get_tuning_config().profiled_turn_pid.to_pid_constants();
    }
    
    // Turn-in-place feedforward constants
    constexpr double TURN_IN_PLACE_KS = 1.278592;
    constexpr double TURN_IN_PLACE_KV = 0.170242;
    constexpr double TURN_IN_PLACE_KA = 0.012877;
    
    // Or get from config if you prefer:
    // inline double get_turn_kS() { return get_tuning_config().turn_in_place_ff.kS; }
    // inline double get_turn_kV() { return get_tuning_config().turn_in_place_ff.kV; }
    // inline double get_turn_kA() { return get_tuning_config().turn_in_place_ff.kA; }

} // namespace abclib::robot_config