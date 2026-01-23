// estimator_config.hpp
#pragma once

#include "abclib/units/units.hpp"
#include "abclib/field/field_config.hpp"
#include "abclib/measurement/measurement_model.hpp"  // ADD THIS
#include <vector>  // ADD THIS

namespace abclib::estimation
{
    enum class FilterType
    {
        GEOMETRIC,
        EKF
    };

    enum class FilterMode
    {
        FULL,           // Predict + Update with measurements
        PREDICTION_ONLY // Predict only, no corrections
    };

    // ADD THIS STRUCT
    struct DistanceSensorConfig
    {
        IMeasurementModel<units::Length>* sensor = nullptr;
        units::Length offset_x = units::Length::from_inches(0.0);      // Forward offset from tracking center
        units::Length offset_y = units::Length::from_inches(0.0);      // Lateral offset from tracking center
        units::Angle bearing = units::Angle::from_degrees(0);          // Direction sensor faces (robot-relative)
        double blend_factor = 0.2;                                     // Weight for this sensor (0.0-1.0)
        bool enabled = true;
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
            // Process noise
            double process_noise_x = 0.01;
            double process_noise_y = 0.01;
            double process_noise_theta = 0.01;

        } ekf;
    };
}