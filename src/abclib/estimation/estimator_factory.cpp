// estimator_factory.cpp
#include "abclib/estimation/estimator_factory.hpp"
#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include "abclib/estimation/ekf_odometry_estimator.hpp"
#include "abclib/measurement/distance_measurement_model.hpp"
#include "abclib/configs/robot_selection.hpp"
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
        switch (config.type)
        {
        case FilterType::GEOMETRIC:
        {
            // Get hardware sensors
            auto [front_sensor, back_sensor] = robot_config::get_distance_sensors();

            // Get sensor configurations (with geometry but no hardware wired yet)
            auto sensor_configs = robot_config::get_distance_sensor_configs();

            // Wire hardware sensors into configs
            if (sensor_configs.size() > 0 && front_sensor)
            {
                sensor_configs[0].sensor = new DistanceSensorMeasurementModel(front_sensor);
            }
            if (sensor_configs.size() > 1 && back_sensor)
            {
                sensor_configs[1].sensor = new DistanceSensorMeasurementModel(back_sensor);
            }

            return std::make_unique<GeometricOdometryEstimator>(
                vertical_model, 
                horizontal_model, 
                imu_model,
                config.vertical_offset, 
                config.horizontal_offset,
                config.field_config,
                sensor_configs);  // Pass the wired sensor array
        }

        case FilterType::EKF:
        {
            // Get hardware sensors
            auto [front_sensor, back_sensor] = robot_config::get_distance_sensors();

            // Create front distance sensor measurement model (EKF currently uses only front)
            auto front_distance_model = front_sensor ? new DistanceSensorMeasurementModel(front_sensor) : nullptr;

            auto ekf = std::make_unique<EKFOdometryEstimator>(
                vertical_model, 
                horizontal_model, 
                imu_model,
                front_distance_model,
                config.vertical_offset, 
                config.horizontal_offset,
                config.field_config,
                config.mode);

            // Configure process noise
            Eigen::Matrix3d process_noise;
            process_noise << config.ekf.process_noise_x, 0.0, 0.0,
                0.0, config.ekf.process_noise_y, 0.0,
                0.0, 0.0, config.ekf.process_noise_theta;

            ekf->set_process_noise(process_noise);
            return ekf;
        }

        default:
        {
            // Default fallback: geometric with no sensors
            return std::make_unique<GeometricOdometryEstimator>(
                vertical_model, 
                horizontal_model, 
                imu_model,
                config.vertical_offset, 
                config.horizontal_offset,
                config.field_config,
                std::vector<DistanceSensorConfig>());  // Empty sensor array
        }
        }
    }
}