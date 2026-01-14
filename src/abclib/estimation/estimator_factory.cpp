#include "abclib/estimation/estimator_factory.hpp"
#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include "abclib/estimation/ekf_odometry_estimator.hpp"

namespace abclib::estimation
{
    std::unique_ptr<IStateEstimator> create_estimator(
        const EstimatorConfig& config,
        IMeasurementModel<units::Length>* vertical_model,
        IMeasurementModel<units::Length>* horizontal_model,
        IMeasurementModel<units::Angle>* imu_model)
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
                auto ekf = std::make_unique<EKFOdometryEstimator>(
                    vertical_model, 
                    horizontal_model, 
                    imu_model,
                    config.vertical_offset, 
                    config.horizontal_offset);
                
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