#pragma once
#include "abclib/control/pid.hpp"
#include "abclib/hardware/motor_group.hpp"
#include <string>

namespace abclib::config {

    // Track where config came from
    enum class ConfigSource {
        HARDCODED,
        SD_CARD
    };

    // Motor characterization and velocity PID
    struct MotorConfig {
        // Feedforward gains
        double kS;  // Static friction
        double kV;  // Velocity gain
        double kA;  // Acceleration gain
        
        // Velocity PID
        double kPv;
        double kIv;
        double kDv;
        
        // Voltage compensation
        bool enable_voltage_compensation;
        double compensation_nominal;      // Target voltage (usually 12.0)
        double compensation_min_battery;  // When to start compensating
        
        // Convert to your existing motor_group_config type
        hardware::motor_group_config to_motor_group_config() const {
            return hardware::motor_group_config{
                .kS = kS,
                .kV = kV,
                .kA = kA,
                .kPv = kPv,
                .kIv = kIv,
                .kDv = kDv,
                .enable_voltage_compensation = enable_voltage_compensation,
                .compensation_nominal = units::Voltage::from_volts(compensation_nominal),
                .compensation_min_battery = units::Voltage::from_volts(compensation_min_battery)
            };
        }
    };

    // PID constants
    struct PIDConfig {
        double kP;
        double kI;
        double kD;
        
        control::PIDConstants to_pid_constants() const {
            return control::PIDConstants(kP, kI, kD);
        }
    };

    // Turn-in-place feedforward
    struct TurnInPlaceFF {
        double kS;
        double kV;
        double kA;
    };

    // Settlement thresholds (from your Chassis::SettlementConfig)
    struct SettlementConfig {
        double angular_threshold_deg;
        double position_threshold_inches;
        double angular_velocity_threshold;
        double linear_velocity_threshold;
        int settle_count_required;
        
        // You'll implement this conversion later when we integrate with Chassis
        // hardware::Chassis::SettlementConfig to_chassis_settlement() const;
    };

    // Main robot configuration - everything tunable goes here
    struct RobotConfig {
        std::string robot_name;
        std::string version = "1.0";
        ConfigSource source = ConfigSource::HARDCODED;
        
        // Motor configurations
        MotorConfig left_motor;
        MotorConfig right_motor;
        
        // PID controllers
        PIDConfig lateral_pid;
        PIDConfig angular_pid;
        PIDConfig profiled_turn_pid;
        
        // Feedforward
        TurnInPlaceFF turn_in_place_ff;
        
        // Settlement
        SettlementConfig settlement;
    };

} // namespace abclib::config