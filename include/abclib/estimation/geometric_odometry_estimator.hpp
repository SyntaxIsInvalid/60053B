#pragma once
#include "api.h"
#include <optional>
#include "state_estimator.hpp"
#include "abclib/measurement/measurement_model.hpp"
#include "arc_length_differential_drive.hpp"
#include "abclib/units/units.hpp"
#include "pose.hpp"
#include "abclib/field/field_map.hpp"
namespace abclib::estimation
{
    class GeometricOdometryEstimator : public IStateEstimator
    {
    private:
        IMeasurementModel<units::Length> *vertical_model_;
        IMeasurementModel<units::Length> *horizontal_model_;
        IMeasurementModel<units::Angle> *imu_model_;
        IMeasurementModel<units::Length> *distance_sensor_ = nullptr;

        units::Length vertical_offset_;
        units::Length horizontal_offset_;

        bool distance_correction_enabled_ = false;
        double distance_blend_factor_ = 0.2;
        units::Length sensor_offset_forward_ = units::Length::from_inches(0.0);
        units::Length sensor_offset_lateral_ = units::Length::from_inches(0.0);

        Pose current_pose_{};

        field::FieldMap field_map_;

        std::optional<pros::Task> tracking_task_;
        mutable pros::Mutex task_mutex_;
        mutable pros::Mutex pose_mutex_;

    public:
        GeometricOdometryEstimator(
            IMeasurementModel<units::Length> *vertical_model,
            IMeasurementModel<units::Length> *horizontal_model,
            IMeasurementModel<units::Angle> *imu_model,
            units::Length vertical_offset,
            units::Length horizontal_offset,
            const field::FieldConfig &field_config,
            IMeasurementModel<units::Length> *distance_sensor_ = nullptr);

        ~GeometricOdometryEstimator();

        void init() override;
        void stop() override;
        void reset() override;
        void calibrate() override;
        void set_pose(const Pose &pose) override;
        Pose get_pose() const override;
        void update() override;
        void enable_distance_correction(bool enable)
        {
            distance_correction_enabled_ = enable;
        }

        void set_distance_blend_factor(double factor)
        {
            distance_blend_factor_ = factor;
        }

        void set_distance_sensor_offset(units::Length forward, units::Length lateral)
        {
            sensor_offset_forward_ = forward;
            sensor_offset_lateral_ = lateral;
        }

        bool is_distance_correction_enabled() const
        {
            return distance_correction_enabled_;
        }

        void apply_distance_correction();

    };
}