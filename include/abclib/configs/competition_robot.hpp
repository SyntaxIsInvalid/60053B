#pragma once
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/motor_tracking_wheel.hpp"
#include "abclib/units/units.hpp"
#include "api.h"
#include "abclib/control/ramsete.hpp"
#include "abclib/estimation/estimator_config.hpp"
#include "abclib/hardware/chassis.hpp"
#include "abclib/control/pid/pid.hpp"

namespace abclib::robot_config
{
    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-11, -13, 12};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {19, 20, -16};
    inline const std::vector<int8_t> TOP_INTAKE_PORTS = {-1};
    inline const std::vector<int8_t> BOTTOM_INTAKE_PORTS = {-10};

    // Sensor ports
    constexpr int8_t IMU_PORT = 17;
    constexpr int8_t Y_ROTATION_PORT = -18;
    constexpr int8_t FRONT_DISTANCE_SENSOR_PORT = 3;
    constexpr int8_t BACK_DISTANCE_SENSOR_PORT = 2;

    // Pneumatic ports
    constexpr char MATCH_LOAD_RAMP_PORT = 'G';
    constexpr char HOOD_PORT = 'H';
    constexpr char WING_PORT = 'F';

    // Intake voltages
    inline constexpr units::Voltage TOP_INTAKE_VOLTAGE = units::Voltage::from_volts(12.0);
    inline constexpr units::Voltage TOP_OUTTAKE_VOLTAGE = units::Voltage::from_volts(-12.0);
    inline constexpr units::Voltage BOTTOM_INTAKE_VOLTAGE = units::Voltage::from_volts(12.0);
    inline constexpr units::Voltage BOTTOM_OUTTAKE_VOLTAGE = units::Voltage::from_volts(-12.0);

    // Physical dimensions
    inline constexpr units::Length WHEEL_DIAMETER = units::Length::from_inches(3.25);
    inline constexpr units::Length TRACK_WIDTH = units::Length::from_inches(27.0);
    inline constexpr units::Length Y_TRACKER_WHEEL_DIAMETER = units::Length::from_inches(2.0);
    inline constexpr units::Length Y_TRACKER_OFFSET = units::Length::from_inches(7.5);

    inline std::vector<pros::Distance *> get_distance_sensors()
    {
        static pros::Distance front_sensor(FRONT_DISTANCE_SENSOR_PORT);
        static pros::Distance back_sensor(BACK_DISTANCE_SENSOR_PORT);

        return {&front_sensor, &back_sensor};
    }

    // Distance sensor configurations with mounting geometry
    inline std::vector<estimation::DistanceSensorConfig> get_distance_sensor_configs()
    {
        return {
            // Front sensor - 3.35" ahead of tracking center, facing forward
            {
                .sensor = nullptr, // Will be filled by factory
                .offset_x = units::Length::from_inches(3.35),
                .offset_y = units::Length::from_inches(0.0),
                .bearing = units::Angle::from_degrees(0), // Forward
                .blend_factor = 0.2,
                .enabled = false},
            // Add more sensors here as you add hardware...
        };
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
            .position_threshold = units::Length::from_inches(0.6),
            .angular_velocity_threshold = units::AngularVelocity::from_rad_per_sec(0.1),
            .linear_velocity_threshold = units::Velocity::from_ips(0.15),
            .settle_count_required = 3};
    }

    inline hardware::ControllerConfig get_controller_config()
    {
        return hardware::ControllerConfig{
            .lateral_pid = {0.67, 0, 0.025},
            .angular_pid = {30, 0, 2.2},
            .profiled_turn_pid = {25, 0.0, 0.0},
            .profiled_lateral_pid = {2.0, 0.0, 0.0},
            .turn_in_place_kS = 0.0,
            .turn_in_place_kV = 0.0,
            .turn_in_place_kA = 0.0,
            .lateral_kS = 0.0,
            .lateral_kV = 0.0,
            .lateral_kA = 0.0,
            .settlement = get_settlement_config(),
            .ramsete = {2.0, 0.7}};
    }

    inline estimation::EstimatorConfig get_estimator_config()
    {
        estimation::EstimatorConfig config;
        config.type = estimation::FilterType::GEOMETRIC;
        config.mode = estimation::FilterMode::PREDICTION_ONLY;
        config.field_config = field::FieldConfig::custom(
            units::Length::from_inches(24), // width
            units::Length::from_inches(48)  // height
        );
        return config;
    }
}