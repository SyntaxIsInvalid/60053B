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

    inline control::PID make_pid(
        const control::PIDConstants &constants,
        const std::optional<filters::FilterConfig> &filter_config = std::nullopt)
    {
        if (filter_config.has_value())
        {
            return control::PID(constants, *filter_config);
        }
        return control::PID(constants);
    }

    struct CalibrationData
    {
        struct
        {
            double kS, kV, kA;
        } drivetrain;

        // Add other subsystems as needed:
        // struct { double kS, kV, kA; } intake;
        // struct { double kS, kV, kA; } lift;
    };

    inline constexpr CalibrationData sysid_data = {
        .drivetrain = {
            .kS = 0.535278,
            .kV = 0.158462,
            .kA = 0.012848}};

    // Motor ports
    inline const std::vector<int8_t> LEFT_MOTOR_PORTS = {-10, 4};
    inline const std::vector<int8_t> RIGHT_MOTOR_PORTS = {6, -11};

    // Sensor ports
    constexpr int8_t IMU_PORT = 8;
    constexpr int8_t Y_ROTATION_PORT = -13;
    constexpr int8_t FRONT_DISTANCE_SENSOR_PORT = 16;
    constexpr int8_t BACK_DISTANCE_SENSOR_PORT = 14;

    // Physical dimensions
    inline constexpr units::Length WHEEL_DIAMETER = units::Length::from_inches(3.25);
    inline constexpr units::Length TRACK_WIDTH = units::Length::from_inches(14.0);
    inline constexpr units::Length Y_TRACKER_WHEEL_DIAMETER = units::Length::from_inches(2.0);
    inline constexpr units::Length Y_TRACKER_OFFSET = units::Length::from_inches(0);

    inline std::vector<pros::Distance *> get_distance_sensors()
    {
        static pros::Distance front_sensor(FRONT_DISTANCE_SENSOR_PORT);
        static pros::Distance back_sensor(BACK_DISTANCE_SENSOR_PORT);

        return {&front_sensor, &back_sensor};
    }

    inline std::vector<estimation::DistanceSensorConfig> get_distance_sensor_configs()
    {
        return {
            {.sensor = nullptr,
             .offset_forward = units::Length::from_inches(3),
             .offset_lateral = units::Length::from_inches(0.0),
             .bearing = units::Angle::from_degrees(0),
             .blend_factor = 0.2,  // Fallback if Kalman disabled
             .enabled = false},
            {.sensor = nullptr,
             .offset_forward = units::Length::from_inches(-5),
             .offset_lateral = units::Length::from_inches(0.0),
             .bearing = units::Angle::from_degrees(180),
             .blend_factor = 0.2,  // Fallback if Kalman disabled
             .enabled = false}};
    }

    inline hardware::motor_group_config get_left_motor_config()
    {
        return hardware::motor_group_config{
            .kS = sysid_data.drivetrain.kS,
            .kV = sysid_data.drivetrain.kV,
            .kA = sysid_data.drivetrain.kA,
            .kPv = 0.2,
            .kIv = 0.0,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)};
    }

    inline hardware::motor_group_config get_right_motor_config()
    {
        return hardware::motor_group_config{
            .kS = sysid_data.drivetrain.kS,
            .kV = sysid_data.drivetrain.kV,
            .kA = sysid_data.drivetrain.kA,
            .kPv = 0.2,
            .kIv = 0.0,
            .kDv = 0.00,
            .enable_voltage_compensation = true,
            .compensation_nominal = units::Voltage::from_volts(12.0),
            .compensation_min_battery = units::Voltage::from_volts(11.5)};
    }

    inline hardware::SettlementConfig get_settlement_config()
    {
        return hardware::SettlementConfig{
            .angular_threshold = units::Angle::from_degrees(0.7),
            .position_threshold = units::Length::from_inches(0.5),
            .angular_velocity_threshold = units::AngularVelocity::from_rad_per_sec(0.1),
            .linear_velocity_threshold = units::Velocity::from_ips(0.15),
            .settle_count_required = 3};
    }

    inline hardware::ControllerConfig get_controller_config()
    {
        return hardware::ControllerConfig{
            // Lateral PID - no derivative, no filtering needed
            .lateral_pid = {
                .kP = 0.6,
                .kI = 0,
                .kD = 0, // ← No derivative term
                .integral = {
                    .max = control::calculate_max_integral_for_voltage(
                        units::Voltage::from_volts(12.0),
                        5)}},

            // Angular PID - has derivative, enable filtering
            .angular_pid = {.kP = 23, .kI = 0,
                            .kD = 2, // ← Has derivative, IMU is noisy
                            .integral = {.max = control::calculate_max_integral_for_voltage(units::Voltage::from_volts(12.0), 50)}},

            // Filter configurations
            .lateral_filter = filters::FilterConfig{.time_constant = 0.015, // 15ms - smooth IMU derivative noise
                                                    .enabled = true},

            .angular_filter = filters::FilterConfig{.time_constant = 0.015, // 15ms - smooth IMU derivative noise
                                                    .enabled = true},

            .profiled_turn_pid = {25, 0.0, 0.0},
            .profiled_lateral_pid = {2.0, 0.0, 0.0},

            .turn_in_place_kS = 1.278592,
            .turn_in_place_kV = 0.170242,
            .turn_in_place_kA = 0.012877,
            .lateral_kS = sysid_data.drivetrain.kS,
            .lateral_kV = sysid_data.drivetrain.kV,
            .lateral_kA = sysid_data.drivetrain.kA,

            .settlement = get_settlement_config(),
            .ramsete = {2.0, 0.7}};
    }

    inline estimation::EstimatorConfig get_estimator_config()
    {
        estimation::EstimatorConfig config;
        config.type = estimation::FilterType::BLENDED_GEOMETRIC;

        // Odometry configuration
        config.vertical_offset = Y_TRACKER_OFFSET;
        config.horizontal_offset = units::Length::from_inches(0.0);
        config.field_config = field::FieldConfig::custom(
            units::Length::from_inches(24),
            units::Length::from_inches(48));

        // Process noise configuration
        config.blending.process_noise.position_noise = units::Length::from_mm(20.0);         // Base drift: 10mm per update (100Hz)
        config.blending.process_noise.heading_noise = units::Angle::from_degrees(0.57);      // ~0.01 rad per update
        config.blending.process_noise.velocity_noise_scale_factor = 0.5;                     // +50% noise per m/s of speed
        config.blending.process_noise.initial_position_uncertainty = units::Length::from_mm(20.0);  // 20mm startup uncertainty
        config.blending.process_noise.initial_heading_uncertainty = units::Angle::from_degrees(2.0); // 2° startup uncertainty

        // Blending configuration
        config.blending.enable_blending = true;
        config.blending.blend_mode = estimation::BlendingConfig::BlendMode::KALMAN;  // Enable 1D Kalman gain
        config.blending.require_stationary = false;
        config.blending.max_blend_velocity = units::Velocity::from_ips(8.0);
        config.blending.max_expected_error = units::Length::from_inches(3.0);
        config.blending.max_sensor_reading = units::Length::from_mm(2100.0);
        config.blending.outlier_mode = estimation::BlendingConfig::OutlierRejectionMode::MAHALANOBIS;
        config.blending.mahalanobis_sigma_threshold = 3.0;

        return config;
    }
}