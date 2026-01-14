#pragma once

#include "state_estimator.hpp"
#include "measurement_model.hpp"
#include "estimator_config.hpp"
#include <memory>

namespace abclib::estimation
{
    /**
     * @brief Factory function to create the appropriate state estimator
     * @param config Estimator configuration (filter type and parameters)
     * @param vertical_model Vertical tracking wheel measurement model
     * @param horizontal_model Horizontal tracking wheel measurement model (can be nullptr)
     * @param imu_model IMU measurement model
     * @return Unique pointer to the created estimator
     */
    std::unique_ptr<IStateEstimator> create_estimator(
        const EstimatorConfig& config,
        IMeasurementModel<units::Length>* vertical_model,
        IMeasurementModel<units::Length>* horizontal_model,
        IMeasurementModel<units::Angle>* imu_model);
}