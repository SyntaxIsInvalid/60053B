// estimator_factory.cpp
#include "abclib/estimation/estimator_factory.hpp"
#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include "abclib/estimation/ekf_odometry_estimator.hpp"
#include "abclib/measurement/distance_measurement_model.hpp"
#include "abclib/configs/robot_selection.hpp"
#include <algorithm>
#include <vector>

#ifdef ROBOT_TEST_DRIVE
#include "abclib/configs/test_robot.hpp"
#elif defined(ROBOT_COMPETITION)
#include "abclib/configs/competition_robot.hpp"
#else
#error "No robot configuration selected!"
#endif

namespace abclib::estimation
{
    std::unique_ptr<IStateEstimator> create_estimator(
        const EstimatorConfig &config,
        IMeasurementModel<units::Length> *vertical_model,
        IMeasurementModel<units::Length> *horizontal_model,
        IMeasurementModel<units::Angle> *imu_model)
    {
        // Common sensor array setup (used by both Geometric and EKF)
        auto build_sensor_array = [&]() -> std::vector<DistanceSensorConfig>
        {
            // Get sensor configurations (geometry only, no hardware yet)
            auto sensor_configs = robot_config::get_distance_sensor_configs();
            
            // If no sensors configured, return empty
            if (sensor_configs.empty())
            {
                return sensor_configs;
            }
            
            // Get hardware sensors from robot config
            auto hardware_sensors = robot_config::get_distance_sensors();
            
            // Wire hardware sensors into configs (pair has exactly 2 elements)
            if (sensor_configs.size() >= 1 && hardware_sensors.first)
            {
                sensor_configs[0].sensor = new DistanceSensorMeasurementModel(hardware_sensors.first);
            }
            if (sensor_configs.size() >= 2 && hardware_sensors.second)
            {
                sensor_configs[1].sensor = new DistanceSensorMeasurementModel(hardware_sensors.second);
            }
            
            return sensor_configs;
        };

        switch (config.type)
        {
        case FilterType::GEOMETRIC:
        {
            auto sensor_configs = build_sensor_array();
            return std::make_unique<GeometricOdometryEstimator>(
                vertical_model,
                horizontal_model,
                imu_model,
                config.vertical_offset,
                config.horizontal_offset,
                config.field_config,
                sensor_configs);
        }
        case FilterType::EKF:
        {
            auto sensor_configs = build_sensor_array();
            auto ekf = std::make_unique<EKFOdometryEstimator>(
                vertical_model,
                horizontal_model,
                imu_model,
                config.vertical_offset,
                config.horizontal_offset,
                config.field_config,
                sensor_configs,
                config.mode,
                config.ekf.measurement_noise);

            // Configure process noise from config
            Eigen::Matrix3d process_noise;
            process_noise << config.ekf.process_noise_x, 0.0, 0.0,
                0.0, config.ekf.process_noise_y, 0.0,
                0.0, 0.0, config.ekf.process_noise_theta;
            ekf->set_process_noise(process_noise);

            return ekf;
        }
        default:
        {
            return std::make_unique<GeometricOdometryEstimator>(
                vertical_model,
                horizontal_model,
                imu_model,
                config.vertical_offset,
                config.horizontal_offset,
                config.field_config,
                std::vector<DistanceSensorConfig>());
        }
        }
    }
}