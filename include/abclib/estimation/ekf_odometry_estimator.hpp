#pragma once

#include "api.h"
#include <optional>
#include "state_estimator.hpp"
#include "measurement_model.hpp"
#include "arc_length_differential_drive.hpp"
#include "abclib/units/units.hpp"
#include "pose.hpp"
#include "abclib/filters/ekf.hpp"
#include "distance_measurement_model.hpp" // ADD THIS
#include "abclib/estimation/estimator_config.hpp"
namespace abclib::estimation
{
    class EKFOdometryEstimator : public IStateEstimator
    {
    private:
        // Measurement models (odometry)
        IMeasurementModel<units::Length> *vertical_model_;
        IMeasurementModel<units::Length> *horizontal_model_;
        IMeasurementModel<units::Angle> *imu_model_;

        units::Length vertical_offset_;
        units::Length horizontal_offset_;

        // State: [x, y, theta] in SI units (meters, radians)
        // Measurements: [front_distance, back_distance] - CHANGE FROM <3,0> TO <3,2>
        filters::ExtendedKalmanFilter<3, 0> ekf_; // CHANGED THIS LINE

        Pose current_pose_{};

        std::optional<pros::Task> tracking_task_;
        mutable pros::Mutex task_mutex_;
        mutable pros::Mutex pose_mutex_;

        // Store previous measurements for delta calculation
        units::Length prev_vertical_total_;
        units::Length prev_horizontal_total_;
        units::Angle prev_imu_total_;
        bool first_update_;
        int update_count_;                         // Track number of updates
        static constexpr int WARMUP_UPDATES = 100; // 1 second at 100Hz
        FilterMode mode_;

    public:
        // UPDATE CONSTRUCTOR SIGNATURE
        EKFOdometryEstimator(
            IMeasurementModel<units::Length> *vertical_model,
            IMeasurementModel<units::Length> *horizontal_model,
            IMeasurementModel<units::Angle> *imu_model,
            units::Length vertical_offset,
            units::Length horizontal_offset,
            FilterMode mode = FilterMode::PREDICTION_ONLY);

        ~EKFOdometryEstimator();

        void init() override;
        void stop() override;
        void reset() override;
        void calibrate() override;
        void set_pose(const Pose &pose) override;
        Pose get_pose() const override;
        void update() override;

        // ADD THIS - for tuning process noise externally
        void set_process_noise(const Eigen::Matrix3d &Q)
        {
            ekf_.set_process_noise(Q);
        }

        void set_mode(FilterMode mode) { mode_ = mode; }
        FilterMode get_mode() const { return mode_; }

    private:
        // Setup initial EKF parameters
        void initialize_ekf();
    };
}