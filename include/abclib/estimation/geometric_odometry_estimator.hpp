// geometric_odometry_estimator.hpp
#pragma once
#include "api.h"
#include <optional>
#include <vector>  // ADD THIS
#include "state_estimator.hpp"
#include "abclib/measurement/measurement_model.hpp"
#include "arc_length_differential_drive.hpp"
#include "abclib/units/units.hpp"
#include "pose.hpp"
#include "abclib/field/field_map.hpp"
#include "estimator_config.hpp"  // ADD THIS for DistanceSensorConfig

namespace abclib::estimation
{
    class GeometricOdometryEstimator : public IStateEstimator
    {
    private:
        IMeasurementModel<units::Length> *vertical_model_;
        IMeasurementModel<units::Length> *horizontal_model_;
        IMeasurementModel<units::Angle> *imu_model_;
        
        // NEW: Multi-sensor support
        std::vector<DistanceSensorConfig> distance_sensors_;

        units::Length vertical_offset_;
        units::Length horizontal_offset_;

        bool distance_correction_enabled_ = false;

        Pose current_pose_{};

        field::FieldMap field_map_;

        std::optional<pros::Task> tracking_task_;
        mutable pros::Mutex task_mutex_;
        mutable pros::Mutex pose_mutex_;

    public:
        // NEW constructor signature:
        GeometricOdometryEstimator(
            IMeasurementModel<units::Length> *vertical_model,
            IMeasurementModel<units::Length> *horizontal_model,
            IMeasurementModel<units::Angle> *imu_model,
            units::Length vertical_offset,
            units::Length horizontal_offset,
            const field::FieldConfig &field_config,
            const std::vector<DistanceSensorConfig>& distance_sensors = {});

        ~GeometricOdometryEstimator();

        void init() override;
        void stop() override;
        void reset() override;
        void calibrate() override;
        void set_pose(const Pose &pose) override;
        Pose get_pose() const override;
        void update() override;
        
        // Distance correction control
        void enable_distance_correction(bool enable)
        {
            distance_correction_enabled_ = enable;
        }

        bool is_distance_correction_enabled() const
        {
            return distance_correction_enabled_;
        }

        // NEW multi-sensor methods:
        void enable_sensor(size_t index, bool enable);
        void set_sensor_blend_factor(size_t index, double factor);
        void set_sensor_config(size_t index, 
                              units::Length offset_x,
                              units::Length offset_y, 
                              units::Angle bearing,
                              double blend_factor);
        size_t get_sensor_count() const { return distance_sensors_.size(); }

    private:
        void apply_distance_correction();
    };
}