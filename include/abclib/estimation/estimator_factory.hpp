#pragma once

#include "state_estimator.hpp"
#include "abclib/measurement/measurement_model.hpp"
#include "estimator_config.hpp"
#include "api.h"  // ADD THIS for pros::Distance
#include <memory>

namespace abclib::estimation
{
    std::unique_ptr<IStateEstimator> create_estimator(
        const EstimatorConfig& config,
        IMeasurementModel<units::Length>* vertical_model,
        IMeasurementModel<units::Length>* horizontal_model,
        IMeasurementModel<units::Angle>* imu_model);
}