#pragma once
#include "api.h"
#include <optional>
#include "state_estimator.hpp"
#include "measurement_model.hpp"
#include "arc_length_differential_drive.hpp"
#include "abclib/units/units.hpp"
#include "pose.hpp"
namespace abclib::estimation
{
    class GeometricOdometryEstimator : public IStateEstimator
    {
    private:
        IMeasurementModel<units::Distance>* vertical_model_;
        IMeasurementModel<units::Distance>* horizontal_model_;
        IMeasurementModel<units::Radians>* imu_model_;
        
        units::Distance vertical_offset_;
        units::Distance horizontal_offset_;

        Pose current_pose_{};

        std::optional<pros::Task> tracking_task_;
        mutable pros::Mutex task_mutex_;
        mutable pros::Mutex pose_mutex_;

    public:
        GeometricOdometryEstimator(
            IMeasurementModel<units::Distance>* vertical_model,
            IMeasurementModel<units::Distance>* horizontal_model,
            IMeasurementModel<units::Radians>* imu_model,
            units::Distance vertical_offset,
            units::Distance horizontal_offset);
            
        ~GeometricOdometryEstimator();

        void init() override;
        void stop() override;
        void reset() override;
        void calibrate() override;
        void set_pose(const Pose& pose) override;
        Pose get_pose() const override;
        void update() override;
    };
}