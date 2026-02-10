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
        BLENDED_GEOMETRIC
    };

    struct DistanceSensorConfig
    {
        IMeasurementModel<units::Length> *sensor = nullptr;
        
        // Sensor mounting position in BODY FRAME:
        // - forward_offset: Y-coordinate (forward/back)
        // - lateral_offset: X-coordinate (LEFT is positive, RIGHT is negative)
        //
        // USAGE EXAMPLES:
        // Right sensor at body (X=4, Y=3):
        //   forward_offset = 3"  (Y in body frame)
        //   lateral_offset = -4" (X in body frame, right is negative)
        //
        // Left sensor at body (X=-4, Y=1):
        //   forward_offset = 1"  (Y in body frame)
        //   lateral_offset = 4"  (X in body frame, left is positive)
        
        units::Length offset_forward = units::Length::from_inches(0.0);
        units::Length offset_lateral = units::Length::from_inches(0.0);
        units::Angle bearing = units::Angle::from_degrees(0);
        double blend_factor = 0.2;
        bool enabled = true;
    };

    struct BlendingConfig
    {
        // Validation thresholds
        units::Length max_sensor_reading = units::Length::from_mm(2100.0);
        units::Length max_expected_error = units::Length::from_inches(3.0);
        units::Velocity max_blend_velocity = units::Velocity::from_ips(10.0);

        // Outlier rejection mode
        enum class OutlierRejectionMode
        {
            SIMPLE_THRESHOLD,
            MAHALANOBIS
        };
        
        OutlierRejectionMode outlier_mode = OutlierRejectionMode::SIMPLE_THRESHOLD;
        double mahalanobis_sigma_threshold = 3.0;

        // Blending behavior
        bool enable_blending = true;
        bool require_stationary = false;
    };

    struct ProcessNoiseConfig
    {
        // Odometry drift uncertainty (standard deviation per 10ms update)
        units::Length position_noise = units::Length::from_mm(10.0);
        units::Angle heading_noise = units::Angle::from_degrees(0.57);
        
        // Initial uncertainty when estimator starts
        units::Length initial_position_uncertainty = units::Length::from_mm(20.0);
        units::Angle initial_heading_uncertainty = units::Angle::from_degrees(1.0);
        
        // Get variance for process noise covariance matrix
        double position_variance_m2() const 
        {
            double sigma_m = position_noise.to_meters();
            return sigma_m * sigma_m;
        }
        
        double heading_variance_rad2() const
        {
            double sigma_rad = heading_noise.to_radians();
            return sigma_rad * sigma_rad;
        }
        
        // Get variance for initial covariance matrix
        double initial_position_variance_m2() const
        {
            double sigma_m = initial_position_uncertainty.to_meters();
            return sigma_m * sigma_m;
        }
        
        double initial_heading_variance_rad2() const
        {
            double sigma_rad = initial_heading_uncertainty.to_radians();
            return sigma_rad * sigma_rad;
        }
    };

    struct EstimatorConfig
    {
        FilterType type = FilterType::GEOMETRIC;

        // Odometry configuration
        units::Length vertical_offset;
        units::Length horizontal_offset;

        // Field geometry
        field::FieldConfig field_config = field::FieldConfig::standard_vex();

        // Process noise (used by BLENDED_GEOMETRIC)
        ProcessNoiseConfig process_noise;

        // Blending-specific parameters (only used when type == BLENDED_GEOMETRIC)
        BlendingConfig blending;
    };
}