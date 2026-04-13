#pragma once

#include "abclib/estimation/state_estimator.hpp"
#include "abclib/estimation/estimator_config.hpp"
#include "abclib/field/field_map.hpp"
#include "abclib/math/xoroshiro128plus.hpp"
#include "pros/rtos.hpp"
#include <vector>
#include <optional>

namespace abclib::estimation
{
    class ParticleFilterEstimator : public IStateEstimator
    {
    private:
        // Measurement models (same pattern as BlendedGeometricEstimator)
        IMeasurementModel<units::Length>* vertical_model_;
        IMeasurementModel<units::Length>* horizontal_model_;
        IMeasurementModel<units::Angle>*  imu_model_;

        // Sensor configs (same as blended - offsets, bearing, etc.)
        std::vector<DistanceSensorConfig> distance_sensors_;

        // Field and config
        field::FieldMap field_map_;
        ParticleFilterConfig pf_config_;
        units::Length vertical_offset_;
        units::Length horizontal_offset_;

        // Current best pose estimate
        Pose current_pose_;

        // RNG
        math::Xoshiro128Plus rng_;

        // Particle arrays (heap allocated, SoA layout)
        std::vector<float> particle_x_;
        std::vector<float> particle_y_;
        std::vector<float> particle_weights_;

        // Temp arrays for resampling
        std::vector<float> temp_x_;
        std::vector<float> temp_y_;
        std::vector<float> temp_weights_;

        // Presample arrays for debugging/logging
        std::vector<float> presample_x_;
        std::vector<float> presample_y_;
        std::vector<float> presample_weights_;

        // Threading (same pattern as BlendedGeometricEstimator)
        mutable pros::Mutex pose_mutex_;
        mutable pros::Mutex task_mutex_;
        std::optional<pros::Task> tracking_task_;

        // Internal steps
        void predict(float dx, float dy);
        void update_weights();
        void resample();
        Pose estimate() const;

        // Odometry
        void update_odometry();
        void update_sensor_weights();

    public:
        ParticleFilterEstimator(
            IMeasurementModel<units::Length>* vertical_model,
            IMeasurementModel<units::Length>* horizontal_model,
            IMeasurementModel<units::Angle>*  imu_model,
            const std::vector<DistanceSensorConfig>& distance_sensors,
            const EstimatorConfig& config);

        ~ParticleFilterEstimator() override;

        // IStateEstimator interface
        void init()      override;
        void stop()      override;
        void reset()     override;
        void calibrate() override;
        void update()    override;

        Pose get_pose()                  const override;
        void set_pose(const Pose& pose)        override;
    };
}