#pragma once

#include "api.h"
#include <optional>
#include "state_estimator.hpp"
#include "abclib/measurement/measurement_model.hpp"
#include "arc_length_differential_drive.hpp"
#include "abclib/units/units.hpp"
#include "pose.hpp"
#include "abclib/filters/ekf.hpp"
#include "abclib/measurement/distance_measurement_model.hpp"
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

        // Distance sensors (NEW: array instead of single sensor)
        std::vector<DistanceSensorConfig> distance_sensors_;
        
        // Field map for wall distance calculations
        field::FieldMap field_map_;

        // State: [x, y, theta] in SI units (meters, radians)
        // Measurements: Variable size based on active sensors
        filters::ExtendedKalmanFilter<3> ekf_;  // CHANGED: removed measurement dim

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
        
        // Measurement noise (from config)
        double measurement_noise_;  // meters (standard deviation)

    public:
        /**
         * @brief Constructor with sensor array (matches Geometric estimator)
         * @param vertical_model Vertical tracking wheel measurement model
         * @param horizontal_model Horizontal tracking wheel measurement model
         * @param imu_model IMU measurement model
         * @param vertical_offset Distance from tracking center to vertical wheel
         * @param horizontal_offset Distance from tracking center to horizontal wheel
         * @param field_config Field dimensions
         * @param distance_sensors Array of distance sensor configurations
         * @param mode Filter mode (FULL or PREDICTION_ONLY)
         * @param measurement_noise Distance sensor noise std dev (meters)
         */
        EKFOdometryEstimator(
            IMeasurementModel<units::Length> *vertical_model,
            IMeasurementModel<units::Length> *horizontal_model,
            IMeasurementModel<units::Angle> *imu_model,
            units::Length vertical_offset,
            units::Length horizontal_offset,
            const field::FieldConfig &field_config,
            const std::vector<DistanceSensorConfig> &distance_sensors,
            FilterMode mode = FilterMode::PREDICTION_ONLY,
            double measurement_noise = 0.05);

        ~EKFOdometryEstimator();

        // IStateEstimator interface
        void init() override;
        void stop() override;
        void reset() override;
        void calibrate() override;
        void set_pose(const Pose &pose) override;
        Pose get_pose() const override;
        void update() override;

        // EKF-specific tuning
        void set_process_noise(const Eigen::Matrix3d &Q)
        {
            ekf_.set_process_noise(Q);
        }

        void set_mode(FilterMode mode) { mode_ = mode; }
        FilterMode get_mode() const { return mode_; }
        
        void set_measurement_noise(double noise_meters) 
        { 
            measurement_noise_ = noise_meters; 
        }
        
        double get_measurement_noise() const 
        { 
            return measurement_noise_; 
        }

        // Sensor configuration (NEW: match Geometric API)
        void enable_sensor(size_t index, bool enable)
        {
            if (index < distance_sensors_.size())
            {
                distance_sensors_[index].enabled = enable;
            }
        }

        void set_sensor_blend_factor(size_t index, double factor)
        {
            if (index < distance_sensors_.size())
            {
                distance_sensors_[index].blend_factor = factor;
            }
        }

        void set_sensor_config(
            size_t index,
            units::Length offset_x,
            units::Length offset_y,
            units::Angle bearing,
            double blend_factor)
        {
            if (index < distance_sensors_.size())
            {
                distance_sensors_[index].offset_x = offset_x;
                distance_sensors_[index].offset_y = offset_y;
                distance_sensors_[index].bearing = bearing;
                distance_sensors_[index].blend_factor = blend_factor;
            }
        }

        size_t get_sensor_count() const
        {
            return distance_sensors_.size();
        }

    private:
        // Setup initial EKF parameters
        void initialize_ekf();

        /**
         * @brief Predict expected distance measurement for a single sensor
         * @param state Current state estimate [x_m, y_m, theta_rad]
         * @param sensor_config Sensor mounting configuration
         * @return Expected distance in meters
         */
        double predict_sensor_distance(
            const Eigen::Vector3d &state,
            const DistanceSensorConfig &sensor_config) const;

        /**
         * @brief Compute numerical Jacobian for a single sensor
         * @param state Current state estimate [x_m, y_m, theta_rad]
         * @param sensor_config Sensor mounting configuration
         * @return Jacobian row [∂h/∂x, ∂h/∂y, ∂h/∂θ]
         */
        Eigen::RowVector3d compute_sensor_jacobian_numerical(
            const Eigen::Vector3d &state,
            const DistanceSensorConfig &sensor_config) const;

        /**
         * @brief Build measurement vector and Jacobian from active sensors
         * @param state Current state estimate
         * @param z_measured Output: measurement vector (resized to N active sensors)
         * @param H_jacobian Output: Jacobian matrix (resized to N×3)
         * @return Number of active sensors used
         */
        int build_measurement_batch(
            const Eigen::Vector3d &state,
            Eigen::VectorXd &z_measured,
            Eigen::MatrixXd &H_jacobian) const;
    };
}