#include "abclib/estimation/estimator_factory.hpp"
#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include "abclib/estimation/ekf_odometry_estimator.hpp"
#include "abclib/measurement/distance_measurement_model.hpp" // ADD THIS
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
        { // ADD THIS BRACE
            // Get distance sensors from robot config
            auto [front_sensor, back_sensor] = robot_config::get_distance_sensors();

            // Create distance sensor measurement model
            auto distance_model = front_sensor ? new DistanceSensorMeasurementModel(front_sensor) : nullptr;

            return std::make_unique<GeometricOdometryEstimator>(
                vertical_model, horizontal_model, imu_model,
                config.vertical_offset, config.horizontal_offset,
                config.field_config,
                distance_model);
        } // ADD THIS BRACE

        case FilterType::EKF:
        { // ADD THIS BRACE
            // Get distance sensors from robot config
            auto [front_sensor, back_sensor] = robot_config::get_distance_sensors();

            // Create distance sensor measurement model
            auto front_distance_model = new DistanceSensorMeasurementModel(front_sensor);

            auto ekf = std::make_unique<EKFOdometryEstimator>(
                vertical_model, horizontal_model, imu_model,
                front_distance_model, // ADD THIS
                config.vertical_offset, config.horizontal_offset,
                config.field_config,
                config.mode);

            // Configure process noise
            Eigen::Matrix3d process_noise;
            process_noise << config.ekf.process_noise_x, 0.0, 0.0,
                0.0, config.ekf.process_noise_y, 0.0,
                0.0, 0.0, config.ekf.process_noise_theta;

            ekf->set_process_noise(process_noise);
            return ekf;
        } // ADD THIS BRACE

        default:
        { // ADD THIS BRACE
            return std::make_unique<GeometricOdometryEstimator>(
                vertical_model, horizontal_model, imu_model,
                config.vertical_offset, config.horizontal_offset,
                config.field_config,
                nullptr); // No distance sensor for default
        } // ADD THIS BRACE
        }
    }

}