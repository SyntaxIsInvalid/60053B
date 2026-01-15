#pragma once

#include "abclib/units/units.hpp"
#include "abclib/field/field_config.hpp"

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
        // Future: UPDATE_ONLY for testing
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