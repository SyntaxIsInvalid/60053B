#pragma once

#include "api.h"
#include <optional>
#include "state_estimator.hpp"
#include "measurement_model.hpp"
#include "arc_length_differential_drive.hpp"
#include "abclib/units/units.hpp"
#include "pose.hpp"
#include "abclib/filters/ekf.hpp"

namespace abclib::estimation
{
    class EKFOdometryEstimator : public IStateEstimator
    {
    private:
        // Measurement models (same as geometric)
        IMeasurementModel<units::Length>* vertical_model_;
        IMeasurementModel<units::Length>* horizontal_model_;
        IMeasurementModel<units::Angle>* imu_model_;
        
        units::Length vertical_offset_;
        units::Length horizontal_offset_;

        // State: [x, y, theta] in SI units (meters, radians)
        // No measurements for now (prediction-only)
        filters::ExtendedKalmanFilter<3, 0> ekf_;

        Pose current_pose_{};

        std::optional<pros::Task> tracking_task_;
        mutable pros::Mutex task_mutex_;
        mutable pros::Mutex pose_mutex_;

        // Store previous measurements for delta calculation
        units::Length prev_vertical_total_;
        units::Length prev_horizontal_total_;
        units::Angle prev_imu_total_;
        bool first_update_;

    public:
        EKFOdometryEstimator(
            IMeasurementModel<units::Length>* vertical_model,
            IMeasurementModel<units::Length>* horizontal_model,
            IMeasurementModel<units::Angle>* imu_model,
            units::Length vertical_offset,
            units::Length horizontal_offset);
            
        ~EKFOdometryEstimator();

        void init() override;
        void stop() override;
        void reset() override;
        void calibrate() override;
        void set_pose(const Pose& pose) override;
        Pose get_pose() const override;
        void update() override;

        void set_process_noise(const Eigen::Matrix3d& process_noise);

    private:
        // Setup initial EKF parameters
        void initialize_ekf();
    };
}