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

namespace abclib::estimation
{
    class EKFOdometryEstimator : public IStateEstimator
    {
    private:
        // Measurement models (odometry)
        IMeasurementModel<units::Length> *vertical_model_;
        IMeasurementModel<units::Length> *horizontal_model_;
        IMeasurementModel<units::Angle> *imu_model_;

        // Distance sensors (for correction) - ADD THESE
        DistanceSensorMeasurementModel *front_distance_sensor_;
        DistanceSensorMeasurementModel *back_distance_sensor_;

        units::Length vertical_offset_;
        units::Length horizontal_offset_;

        // Sensor configuration - ADD THESE
        units::Length front_sensor_offset_forward_;
        units::Length front_sensor_offset_lateral_;
        double front_sensor_bearing_;

        units::Length back_sensor_offset_forward_;
        units::Length back_sensor_offset_lateral_;
        double back_sensor_bearing_;

        double distance_sensor_noise_; // Measurement noise std dev

        // State: [x, y, theta] in SI units (meters, radians)
        // Measurements: [front_distance, back_distance] - CHANGE FROM <3,0> TO <3,2>
        filters::ExtendedKalmanFilter<3, 2> ekf_; // CHANGED THIS LINE

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
        units::Length prev_front_measurement_;
        units::Length prev_back_measurement_;

    public:
        // UPDATE CONSTRUCTOR SIGNATURE
        EKFOdometryEstimator(
            IMeasurementModel<units::Length> *vertical_model,
            IMeasurementModel<units::Length> *horizontal_model,
            IMeasurementModel<units::Angle> *imu_model,
            units::Length vertical_offset,
            units::Length horizontal_offset,
            DistanceSensorMeasurementModel *front_distance_sensor, // ADD
            DistanceSensorMeasurementModel *back_distance_sensor,  // ADD
            units::Length front_sensor_offset_forward,             // ADD
            units::Length front_sensor_offset_lateral,             // ADD
            double front_sensor_bearing,                           // ADD
            units::Length back_sensor_offset_forward,              // ADD
            units::Length back_sensor_offset_lateral,              // ADD
            double back_sensor_bearing,                            // ADD
            double distance_sensor_noise);                         // ADD

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

    private:
        // Setup initial EKF parameters
        void initialize_ekf();
    };
}