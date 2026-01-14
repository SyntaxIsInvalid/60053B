#pragma once

#include "abclib/units/units.hpp"

namespace abclib::estimation
{
    enum class FilterType
    {
        GEOMETRIC,
        EKF
    };

    struct EstimatorConfig
    {
        FilterType type = FilterType::GEOMETRIC;

        // Common parameters (used by all estimators)
        units::Length vertical_offset;
        units::Length horizontal_offset;

        // EKF-specific parameters (only used when type == EKF)
        struct EKFParams
        {
            // Process noise
            double process_noise_x = 0.01;
            double process_noise_y = 0.01;
            double process_noise_theta = 0.01;

            // Distance sensor configuration
            units::Length front_sensor_offset_forward = units::Length::from_inches(0.0);
            units::Length front_sensor_offset_lateral = units::Length::from_inches(0.0);
            double front_sensor_bearing = 0.0;
            
            units::Length back_sensor_offset_forward = units::Length::from_inches(0.0);
            units::Length back_sensor_offset_lateral = units::Length::from_inches(0.0);
            double back_sensor_bearing = M_PI;

            // Measurement noise (standard deviation in inches)
            double distance_sensor_noise = 0.5; // 0.5 inch std dev
        } ekf;
    };
}