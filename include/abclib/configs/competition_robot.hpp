#pragma once
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/tracking_wheel.hpp"
#include "abclib/units/units.hpp"
#include "abclib/configs/config_loader.hpp"
#include "api.h"

namespace abclib::robot_config {

    // ========== HARDWARE CONSTANTS ==========
    
    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-12, -11, -19};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {14, 15, 20};
    inline const std::vector<int8_t> TOP_INTAKE_PORTS = {9};
    inline const std::vector<int8_t> BOTTOM_INTAKE_PORTS = {-10};

    // Intake voltages
    inline const units::Voltage TOP_INTAKE_VOLTAGE = units::Voltage::from_volts(12.0);
    inline const units::Voltage TOP_OUTTAKE_VOLTAGE = units::Voltage::from_volts(-12.0);
    inline const units::Voltage BOTTOM_INTAKE_VOLTAGE = units::Voltage::from_volts(12.0);
    inline const units::Voltage BOTTOM_OUTTAKE_VOLTAGE = units::Voltage::from_volts(-12.0);

    // Sensor ports
    constexpr int8_t IMU_PORT = 18;
    constexpr int8_t Y_ROTATION_PORT = -17;

    // Pneumatic ports
    constexpr char MATCH_LOAD_RAMP_PORT = 'G';
    constexpr char INTAKE_LIFT_PORT = 'H';

    // Physical dimensions
    inline const units::Distance WHEEL_DIAMETER = units::Distance::from_inches(2.75);
    inline const units::Distance TRACK_WIDTH = units::Distance::from_inches(14.0);
    inline const units::Distance Y_TRACKER_WHEEL_DIAMETER = units::Distance::from_inches(2.0);
    inline const units::Distance Y_TRACKER_OFFSET = units::Distance::from_inches(0.0);

    
    // ========== TUNABLE VALUES ==========
    
    inline config::RobotConfig& get_tuning_config() {
        static config::RobotConfig config = 
            config::ConfigLoader::load("competition_robot");
        return config;
    }
    
    inline hardware::motor_group_config get_left_motor_config() {
        return get_tuning_config().left_motor.to_motor_group_config();
    }

    inline hardware::motor_group_config get_right_motor_config() {
        return get_tuning_config().right_motor.to_motor_group_config();
    }

    inline control::PIDConstants get_lateral_pid() {
        return get_tuning_config().lateral_pid.to_pid_constants();
    }

    inline control::PIDConstants get_angular_pid() {
        return get_tuning_config().angular_pid.to_pid_constants();
    }

    inline control::PIDConstants get_profiled_turn_pid() {
        return get_tuning_config().profiled_turn_pid.to_pid_constants();
    }

    // Turn-in-place feedforward
    constexpr double TURN_IN_PLACE_KS = 0.919850;
    constexpr double TURN_IN_PLACE_KV = 0.169750;
    constexpr double TURN_IN_PLACE_KA = 0;

} // namespace abclib::robot_config