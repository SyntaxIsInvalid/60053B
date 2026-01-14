#pragma once

#include "abclib/units/units.hpp"

namespace abclib::estimation
{
    enum class FilterType {
        GEOMETRIC,
        EKF
    };
    
    struct EstimatorConfig {
        FilterType type = FilterType::GEOMETRIC;
        
        // Common parameters (used by all estimators)
        units::Length vertical_offset;
        units::Length horizontal_offset;
        
        // EKF-specific parameters (only used when type == EKF)
        struct EKFParams {
            double process_noise_x = 0.01;
            double process_noise_y = 0.01;
            double process_noise_theta = 0.01;
        } ekf;
    };
}