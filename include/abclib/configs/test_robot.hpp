#pragma once
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/motor_tracking_wheel.hpp"
#include "abclib/units/units.hpp"
#include "api.h"
#include "abclib/control/ramsete.hpp"
#include "abclib/estimation/estimator_config.hpp"
#include "abclib/hardware/chassis.hpp"

namespace abclib::robot_config
{

    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-10, 4};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {6, -11};

    // Sensor ports
    constexpr int8_t IMU_PORT = 9;
    constexpr int8_t FRONT_DISTANCE_SENSOR_PORT = 12; // YOUR ACTUAL PORT
    constexpr int8_t BACK_DISTANCE_SENSOR_PORT = 1;   // YOUR ACTUAL PORT

    // Physical dimensions (using typed units)
    inline constexpr units::Length WHEEL_DIAMETER = units::Length::from_inches(3.25);
    inline constexpr units::Length TRACK_WIDTH = units::Length::from_inches(14.0);
    inline constexpr units::Length Y_TRACKER_OFFSET = units::Length::from_inches(7.0);

    inline std::pair<pros::Distance *, pros::Distance *> get_distance_sensors()
    {
        static pros::Distance front_sensor(FRONT_DISTANCE_SENSOR_PORT);
        static pros::Distance back_sensor(BACK_DISTANCE_SENSOR_PORT);

        return {&front_sensor, &back_sensor};
    }

    // Motor configurations
    inline hardware::motor_group_config get_left_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 0.535278,
            .kV = 0.158462,
            .kA = 0.012848,
            .kPv = 0.0,
            .kIv = 0.25,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)};
    }

    inline hardware::motor_group_config get_right_motor_config()
    {
        return hardware::motor_group_config{
            .kS = 0.535278,
            .kV = 0.158462,
            .kA = 0.012848,
            .kPv = 0.0,
            .kIv = 0.0,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)};
    }

    inline hardware::SettlementConfig get_settlement_config()
    {
        return hardware::SettlementConfig{
            .angular_threshold = units::Angle::from_degrees(1),
            .position_threshold = units::Length::from_inches(0.5),
            .angular_velocity_threshold = units::AngularVelocity::from_rad_per_sec(0.1),
            .linear_velocity_threshold = units::Velocity::from_ips(0.15),
            .settle_count_required = 3};
    }

    inline hardware::ControllerConfig get_controller_config()
    {
        return hardware::ControllerConfig{
            .lateral_pid = {0.6, 5, 0},
            .angular_pid = {5, 50, 0},
            .profiled_turn_pid = {25, 0.0, 0.0},
            .profiled_lateral_pid = {2.0, 0.0, 0.0},
            .turn_in_place_kS = 1.278592,
            .turn_in_place_kV = 0.170242,
            .turn_in_place_kA = 0.012877,
            .lateral_kS = 0.535278,
            .lateral_kV = 0.158462,
            .lateral_kA = 0.012848,
            .settlement = get_settlement_config(), // uses defaults
            .ramsete = {2.0, 0.7}};
    }

    inline estimation::DistanceCorrectionConfig get_distance_correction_config()
    {
        return estimation::DistanceCorrectionConfig{
            .sensor_offset_forward = units::Length::from_inches(3.35), // Adjust to your actual sensor position
            .sensor_offset_lateral = units::Length::from_inches(0), // Adjust to your actual sensor position
            .blend_factor = 0.2                                       // 20% sensor, 80% odometry
        };
    }

    inline estimation::EstimatorConfig get_estimator_config()
    {
        estimation::EstimatorConfig config;
        config.type = estimation::FilterType::GEOMETRIC; // or GEOMETRIC/EKF
        config.mode = estimation::FilterMode::PREDICTION_ONLY;
        config.field_config = field::FieldConfig::custom(
            units::Length::from_inches(24), // width
            units::Length::from_inches(48)  // height
        );
        return config;
    }

}