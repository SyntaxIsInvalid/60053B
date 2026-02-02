// blended_geometric_estimator.hpp
#pragma once
#include "api.h"
#include <optional>
#include <vector>
#include "state_estimator.hpp"
#include "abclib/measurement/measurement_model.hpp"
#include "arc_length_differential_drive.hpp"
#include "abclib/units/units.hpp"
#include "pose.hpp"
#include "abclib/field/field_map.hpp"
#include "estimator_config.hpp"

namespace abclib::estimation
{
    /**
     * @brief Geometric odometry estimator with continuous distance sensor blending
     * 
     * Combines wheel encoder + IMU odometry with opportunistic distance sensor corrections.
     * Sensors continuously validate and correct pose drift while robot is moving.
     * 
     * Initial testing: Single front-facing sensor
     * Production: Multi-sensor support via sensor array
     */
    class BlendedGeometricEstimator : public IStateEstimator
    {
    private:
        // Odometry sensors
        IMeasurementModel<units::Length>* vertical_model_;
        IMeasurementModel<units::Length>* horizontal_model_;
        IMeasurementModel<units::Angle>* imu_model_;
        
        // Odometry configuration
        units::Length vertical_offset_;
        units::Length horizontal_offset_;
        
        // Distance sensors (start with one, expand later)
        std::vector<DistanceSensorConfig> distance_sensors_;
        
        // Field geometry
        field::FieldMap field_map_;
        
        // Blending configuration
        BlendingConfig blend_config_;
        
        // Current state
        Pose current_pose_{};
        
        // Task management
        std::optional<pros::Task> tracking_task_;
        mutable pros::Mutex task_mutex_;
        mutable pros::Mutex pose_mutex_;
        
    public:
        /**
         * @brief Construct blended geometric estimator
         * 
         * @param vertical_model Forward tracking wheel encoder
         * @param horizontal_model Lateral tracking wheel encoder (can be nullptr for diff drive)
         * @param imu_model IMU for heading
         * @param vertical_offset Distance from tracking center to vertical wheel
         * @param horizontal_offset Distance from tracking center to horizontal wheel
         * @param field_config Field dimensions for wall calculations
         * @param distance_sensors Array of distance sensor configurations
         * @param blend_config Blending behavior parameters
         */
        BlendedGeometricEstimator(
            IMeasurementModel<units::Length>* vertical_model,
            IMeasurementModel<units::Length>* horizontal_model,
            IMeasurementModel<units::Angle>* imu_model,
            units::Length vertical_offset,
            units::Length horizontal_offset,
            const field::FieldConfig& field_config,
            const std::vector<DistanceSensorConfig>& distance_sensors,
            const BlendingConfig& blend_config = BlendingConfig{});
        
        ~BlendedGeometricEstimator();
        
        // ============= IStateEstimator Interface =============
        
        void init() override;
        void stop() override;
        void reset() override;
        void calibrate() override;
        void set_pose(const Pose& pose) override;
        Pose get_pose() const override;
        void update() override;
        
        // ============= Configuration =============
        
        /**
         * @brief Update blending configuration at runtime
         */
        void set_blend_config(const BlendingConfig& config);
        
        /**
         * @brief Enable/disable sensor blending
         */
        void set_blending_enabled(bool enabled);
        
    private:
        // ============= Odometry Update =============
        
        /**
         * @brief Update pose using wheel encoders and IMU
         */
        void update_odometry();
        
        // ============= Sensor Blending =============
        
        /**
         * @brief Attempt to apply distance sensor corrections
         * 
         * Iterates through all enabled sensors, validates readings,
         * and applies blended corrections from valid measurements.
         */
        void update_sensor_corrections();
        
        /**
         * @brief Validate a distance sensor reading
         * 
         * @param sensor_config Sensor geometry and settings
         * @param reading Measured distance from sensor
         * @param robot_pose Current robot pose estimate
         * @param[out] expected_distance Computed expected distance to wall
         * @param[out] detected_wall Which wall sensor is facing
         * @return true if reading is valid and should be used
         */
        bool validate_sensor_reading(
            const DistanceSensorConfig& sensor_config,
            units::Length reading,
            const Pose& robot_pose,
            units::Length& expected_distance,
            field::FieldMap::Wall& detected_wall) const;
        
        /**
         * @brief Apply blended correction from distance sensor
         * 
         * Computes position error and applies weighted correction
         * in direction perpendicular to detected wall.
         * 
         * @param sensor_config Sensor configuration (includes blend_factor)
         * @param measured_distance Actual sensor reading
         * @param expected_distance Expected distance based on current pose
         * @param wall Which wall the measurement corresponds to
         */
        void apply_sensor_correction(
            const DistanceSensorConfig& sensor_config,
            units::Length measured_distance,
            units::Length expected_distance,
            field::FieldMap::Wall wall);
        
        /**
         * @brief Check if robot is moving slow enough for safe blending
         */
        bool is_safe_to_blend() const;
    };
}