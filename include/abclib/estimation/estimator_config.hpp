// estimator_config.hpp
#pragma once

#include "abclib/units/units.hpp"
#include "abclib/field/field_config.hpp"
#include "abclib/measurement/measurement_model.hpp"
#include <vector>

namespace abclib::estimation
{
    enum class FilterType
    {
        GEOMETRIC,
        EKF,
        BLENDED_GEOMETRIC // NEW
    };

    enum class FilterMode
    {
        FULL,           // Predict + Update with measurements
        PREDICTION_ONLY // Predict only, no corrections
    };

    struct DistanceSensorConfig
    {
        IMeasurementModel<units::Length> *sensor = nullptr;
        // USAGE EXAMPLES:
        // Right sensor at body position (4, 3):
        //   forward_offset = 3"  (y-coordinate in body frame)
        //   lateral_offset = -4" (x-coordinate in body frame, LEFT is positive, RIGHT is negative)
        //
        // Left sensor at body position (-4, 1):
        //   forward_offset = 1"  (y-coordinate in body frame)
        //   lateral_offset = 4"  (x-coordinate in body frame, LEFT is positive)

        units::Length offset_forward = units::Length::from_inches(0.0); // Forward offset from tracking center
        units::Length offset_lateral = units::Length::from_inches(0.0); // Lateral offset from tracking center
        units::Angle bearing = units::Angle::from_degrees(0);           // Direction sensor faces (robot-relative)
        double blend_factor = 0.2;                                      // Weight for this sensor (0.0-1.0)
        bool enabled = true;
    };

    // NEW: Blending configuration
    struct BlendingConfig
    {
        // Validation thresholds
        units::Length max_sensor_reading = units::Length::from_mm(2100.0);    // 2000mm + 5%
        units::Length max_expected_error = units::Length::from_inches(3.0);   // Reject if off by >3"
        units::Velocity max_blend_velocity = units::Velocity::from_ips(10.0); // Only blend when slow

        // Blending behavior
        bool enable_blending = true;
        bool require_stationary = false; // If true, only blend when v ≈ 0
    };

    struct EstimatorConfig
    {
        FilterType type = FilterType::GEOMETRIC;
        FilterMode mode = FilterMode::PREDICTION_ONLY;

        // Common parameters (used by all estimators)
        units::Length vertical_offset;
        units::Length horizontal_offset;

        field::FieldConfig field_config = field::FieldConfig::standard_vex();

        // EKF-specific parameters (only used when type == EKF)
        struct EKFParams
        {
            // Process noise (standard deviation in SI units)
            // Represents uncertainty in odometry prediction per update cycle
            double process_noise_x = 0.01;     // meters (10mm)
            double process_noise_y = 0.01;     // meters (10mm)
            double process_noise_theta = 0.01; // radians (~0.57 degrees)

            // Measurement noise (standard deviation in SI units)
            // Represents distance sensor accuracy
            // Default: 0.05m = 50mm (conservative mid-range estimate)
            double measurement_noise = 0.05; // meters (50mm)

        } ekf;

        // Blending-specific parameters (only used when type == BLENDED_GEOMETRIC)
        BlendingConfig blending;
    };
}