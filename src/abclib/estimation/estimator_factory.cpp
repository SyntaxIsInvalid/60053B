#include "abclib/estimation/estimator_factory.hpp"
#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include "abclib/estimation/ekf_odometry_estimator.hpp"
#include "abclib/estimation/distance_measurement_model.hpp"  // ADD THIS

namespace abclib::estimation
{
    std::unique_ptr<IStateEstimator> create_estimator(
        const EstimatorConfig& config,
        IMeasurementModel<units::Length>* vertical_model,
        IMeasurementModel<units::Length>* horizontal_model,
        IMeasurementModel<units::Angle>* imu_model,
        pros::Distance* front_distance_sensor,      // ADD THESE PARAMETERS
        pros::Distance* back_distance_sensor)       // ADD THESE PARAMETERS
    {
        switch(config.type) {
            case FilterType::GEOMETRIC:
                return std::make_unique<GeometricOdometryEstimator>(
                    vertical_model, 
                    horizontal_model, 
                    imu_model,
                    config.vertical_offset, 
                    config.horizontal_offset);
                
            case FilterType::EKF: {
                // Create distance sensor measurement models
                auto* front_model = new DistanceSensorMeasurementModel(front_distance_sensor);
                auto* back_model = new DistanceSensorMeasurementModel(back_distance_sensor);
                
                auto ekf = std::make_unique<EKFOdometryEstimator>(
                    vertical_model, 
                    horizontal_model, 
                    imu_model,
                    config.vertical_offset, 
                    config.horizontal_offset,
                    front_model,                                    // ADD
                    back_model,                                     // ADD
                    config.ekf.front_sensor_offset_forward,         // ADD
                    config.ekf.front_sensor_offset_lateral,         // ADD
                    config.ekf.front_sensor_bearing,                // ADD
                    config.ekf.back_sensor_offset_forward,          // ADD
                    config.ekf.back_sensor_offset_lateral,          // ADD
                    config.ekf.back_sensor_bearing,                 // ADD
                    config.ekf.distance_sensor_noise);              // ADD
                
                // Configure EKF-specific process noise
                Eigen::Matrix3d process_noise;
                process_noise << 
                    config.ekf.process_noise_x, 0.0, 0.0,
                    0.0, config.ekf.process_noise_y, 0.0,
                    0.0, 0.0, config.ekf.process_noise_theta;
                
                ekf->set_process_noise(process_noise);
                return ekf;
            }
            
            default:
                // Fallback to geometric if unknown type
                return std::make_unique<GeometricOdometryEstimator>(
                    vertical_model, 
                    horizontal_model, 
                    imu_model,
                    config.vertical_offset, 
                    config.horizontal_offset);
        }
    }
}