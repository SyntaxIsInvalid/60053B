#if 0
#pragma once

#include "state_estimator.hpp"
#include "measurement_model.hpp"
#include "arc_length_differential_drive.hpp"
#include "abclib/filters/ukf.hpp"
#include "abclib/units/units.hpp"
#include "pose.hpp"
#include "api.h"
#include <optional>

namespace abclib::estimation
{
    class UKFEstimator : public IStateEstimator
    {
    private:
        // State: [x, y, theta, v, omega]
        static constexpr int STATE_DIM = 5;
        // Measurements: [delta_forward, delta_lateral, delta_theta]
        static constexpr int MEAS_DIM = 3;
        
        // The actual UKF filter
        filters::UnscentedKalmanFilter<STATE_DIM, MEAS_DIM> ukf_;
        
        // Measurement models (injected, not owned)
        IMeasurementModel<units::Distance>* vertical_model_;
        IMeasurementModel<units::Distance>* horizontal_model_;
        IMeasurementModel<units::Radians>* imu_model_;
        
        // Robot geometry
        units::Distance vertical_offset_;
        units::Distance horizontal_offset_;
        
        // Tracking task
        std::optional<pros::Task> tracking_task_;
        mutable pros::Mutex task_mutex_;
        mutable pros::Mutex pose_mutex_;
        
        // Current pose (synchronized access)
        Pose current_pose_;
        
        // Time tracking for dt calculation
        uint32_t last_update_time_ms_;
        
    public:
        /**
         * @brief Construct UKF estimator
         * 
         * @param vertical_model Vertical tracking wheel measurement model
         * @param horizontal_model Horizontal tracking wheel measurement model (can be nullptr)
         * @param imu_model IMU gyroscope measurement model
         * @param vertical_offset Distance from robot center to vertical wheel
         * @param horizontal_offset Distance from robot center to horizontal wheel
         * @param process_noise_scale Scaling factor for process noise Q (tune this)
         */
        UKFEstimator(
            IMeasurementModel<units::Distance>* vertical_model,
            IMeasurementModel<units::Distance>* horizontal_model,
            IMeasurementModel<units::Radians>* imu_model,
            units::Distance vertical_offset,
            units::Distance horizontal_offset,
            double process_noise_scale = 1.0
        );
        
        ~UKFEstimator() override;
        
        // IStateEstimator interface
        void init() override;
        void stop() override;
        void reset() override;
        void calibrate() override;
        void update() override;
        Pose get_pose() const override;
        void set_pose(const Pose& pose) override;
        
    private:
        /**
         * @brief Initialize process noise covariance Q
         */
        void initialize_process_noise(double scale);
        
        /**
         * @brief Build measurement covariance R from current sensor uncertainties
         */
        Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> build_measurement_covariance();
    };
    
} // namespace abclib::estimation
#endif