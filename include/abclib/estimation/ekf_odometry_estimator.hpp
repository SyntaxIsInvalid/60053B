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
#include "abclib/field/field_map.hpp"
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

        IMeasurementModel<units::Length> *front_distance_model_;

        // Sensor mounting offsets (from tracking center to sensor)
        static constexpr double SENSOR_FORWARD_OFFSET = 3.35; // inches
        static constexpr double SENSOR_LATERAL_OFFSET = 0.0;  // inches
        static constexpr double SENSOR_BEARING = 0.0;         // radians (pointing forward)

        // State: [x, y, theta] in SI units (meters, radians)
        // Measurements: [front_distance, back_distance] - CHANGE FROM <3,0> TO <3,2>
        filters::ExtendedKalmanFilter<3, 1> ekf_; // CHANGED THIS LINE

        Pose current_pose_{};

        std::optional<pros::Task> tracking_task_;
        mutable pros::Mutex task_mutex_;
        mutable pros::Mutex pose_mutex_;

        // Store previous measurements for delta calculation
        units::Length prev_vertical_total_;
        units::Length prev_horizontal_total_;
        units::Angle prev_imu_total_;
        bool first_update_;
        FilterMode mode_;
        field::FieldMap field_map_;

    public:
        // UPDATE CONSTRUCTOR SIGNATURE
        EKFOdometryEstimator(
            IMeasurementModel<units::Length> *vertical_model,
            IMeasurementModel<units::Length> *horizontal_model,
            IMeasurementModel<units::Angle> *imu_model,
            IMeasurementModel<units::Length> *front_distance_model,
            units::Length vertical_offset,
            units::Length horizontal_offset,
            const field::FieldConfig &field_config,
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