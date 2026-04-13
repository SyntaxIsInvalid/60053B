#include "abclib/estimation/particle_filter_estimator.hpp"
#include "abclib/estimation/arc_length_differential_drive.hpp"
#include "abclib/measurement/distance_measurement_model.hpp"
#include "abclib/math/raycast.hpp"
#include "abclib/math/angles.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <mutex>

namespace abclib::estimation
{

    // Internal only - precomputed per-sensor, per-update, never per-particle
    struct SensorReading
    {
        float recorded; // actual sensor reading in inches
        float inv_var;  // -0.5 / (std_dev^2), precomputed for weight calculation
        float rel_x;    // sensor world-frame x position
        float rel_y;    // sensor world-frame y position
        float ray_cos;  // ray direction x (cos of sensor heading in world frame)
        float ray_sin;  // ray direction y (sin of sensor heading in world frame)
    };

    ParticleFilterEstimator::ParticleFilterEstimator(
        IMeasurementModel<units::Length> *vertical_model,
        IMeasurementModel<units::Length> *horizontal_model,
        IMeasurementModel<units::Angle> *imu_model,
        const std::vector<DistanceSensorConfig> &distance_sensors,
        const EstimatorConfig &config)
        : vertical_model_(vertical_model),
          horizontal_model_(horizontal_model),
          imu_model_(imu_model),
          distance_sensors_(distance_sensors),
          field_map_(config.field_config),
          pf_config_(config.particle_filter),
          vertical_offset_(config.vertical_offset),
          horizontal_offset_(config.horizontal_offset),
          rng_(pros::micros())
    {
        const size_t N = pf_config_.num_particles;

        // Allocate particle arrays
        particle_x_.resize(N, 0.0f);
        particle_y_.resize(N, 0.0f);
        particle_weights_.resize(N, 1.0f / static_cast<float>(N));

        // Allocate temp arrays for resampling
        temp_x_.resize(N, 0.0f);
        temp_y_.resize(N, 0.0f);
        temp_weights_.resize(N, 0.0f);

        // Allocate presample arrays for debugging
        if (pf_config_.keep_presample_arrays)
        {
            presample_x_.resize(N, 0.0f);
            presample_y_.resize(N, 0.0f);
            presample_weights_.resize(N, 0.0f);
        }
    }

    ParticleFilterEstimator::~ParticleFilterEstimator()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task_.has_value())
        {
            tracking_task_->notify();
        }
    }

    void ParticleFilterEstimator::init()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);

        if (!tracking_task_.has_value())
        {
            tracking_task_ = pros::Task([this]
                                        {
            while (pros::Task::notify_take(true, 0) == 0)
            {
                this->update();
                pros::delay(10);
            } });
        }
    }

    void ParticleFilterEstimator::stop()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task_.has_value())
        {
            tracking_task_->notify();
            tracking_task_ = std::nullopt;
        }
    }

    void ParticleFilterEstimator::reset()
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);

        current_pose_ = Pose();

        const size_t N = pf_config_.num_particles;
        const float uniform = 1.0f / static_cast<float>(N);

        // Reset all particles to origin with uniform weights
        std::fill(particle_x_.begin(), particle_x_.end(), 0.0f);
        std::fill(particle_y_.begin(), particle_y_.end(), 0.0f);
        std::fill(particle_weights_.begin(), particle_weights_.end(), uniform);

        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();
    }

    void ParticleFilterEstimator::calibrate()
    {
        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();

        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        current_pose_ = Pose();

        const size_t N = pf_config_.num_particles;
        const float uniform = 1.0f / static_cast<float>(N);

        std::fill(particle_x_.begin(), particle_x_.end(), 0.0f);
        std::fill(particle_y_.begin(), particle_y_.end(), 0.0f);
        std::fill(particle_weights_.begin(), particle_weights_.end(), uniform);
    }

    Pose ParticleFilterEstimator::get_pose() const
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        return current_pose_;
    }

    void ParticleFilterEstimator::set_pose(const Pose &pose)
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        current_pose_ = pose;

        // Reinitialize particles around the new pose
        const size_t N = pf_config_.num_particles;
        const float uniform = 1.0f / static_cast<float>(N);
        const float spread = pf_config_.initial_spread;

        const float cx = static_cast<float>(pose.x_inches());
        const float cy = static_cast<float>(pose.y_inches());

        for (size_t i = 0; i < N; i++)
        {
            particle_x_[i] = cx + rng_.gaussian(spread);
            particle_y_[i] = cy + rng_.gaussian(spread);
            particle_weights_[i] = uniform;
        }
    }

    void ParticleFilterEstimator::update()
    {
        update_odometry();
        update_sensor_weights();
    }

    void ParticleFilterEstimator::update_odometry()
    {
        // Get sensor deltas - same pattern as BlendedGeometricEstimator
        units::Length delta_vertical = vertical_model_ ? vertical_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Length delta_horizontal = horizontal_model_ ? horizontal_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Angle delta_imu = imu_model_ ? imu_model_->get_measurement() : units::Angle::from_radians(0.0);

        // Compute local SE2 transformation from odometry
        math::SE2 local_transform = ArcLengthDifferentialDrive::compute_local_transformation(
            delta_vertical,
            delta_horizontal,
            delta_imu,
            vertical_offset_,
            horizontal_offset_);

        // Extract dx/dy as floats - cast once here, never inside particle loop
        Eigen::Vector3d xi = local_transform.log();
        float dx = static_cast<float>(xi(0)); // lateral
        float dy = static_cast<float>(xi(1)); // forward

        // Update robot pose with odometry (for sensor precomputation)
        {
            std::lock_guard<pros::Mutex> lock(pose_mutex_);
            current_pose_.se2 = current_pose_.se2 * local_transform;
        }

        // Run particle filter steps
        predict(dx, dy);
    }

    void ParticleFilterEstimator::predict(float dx, float dy)
    {
        const size_t N = pf_config_.num_particles;
        const float std_dev = std::hypot(dx, dy) * pf_config_.odometry_noise_scale;

        // Hoist bounds out of loop - wall positions never change
        const auto bounds = field_map_.get_wall_bounds_f32();

        for (size_t i = 0; i < N; i++)
        {
            particle_x_[i] += dx + rng_.gaussian(std_dev);
            particle_y_[i] += dy + rng_.gaussian(std_dev);

            particle_x_[i] = std::clamp(particle_x_[i], bounds.west, bounds.east);
            particle_y_[i] = std::clamp(particle_y_[i], bounds.south, bounds.north);
        }
    }

    void ParticleFilterEstimator::update_sensor_weights()
    {
        // Get current pose for sensor precomputation
        Pose robot_pose;
        {
            std::lock_guard<pros::Mutex> lock(pose_mutex_);
            robot_pose = current_pose_;
        }

        // Build SensorReading list - all Eigen/FieldMap work happens here
        // outside the particle loop
        std::vector<SensorReading> readings;
        readings.reserve(distance_sensors_.size());

        for (auto &sensor_config : distance_sensors_)
        {
            if (!sensor_config.enabled)
                continue;

            auto *distance_model = dynamic_cast<DistanceSensorMeasurementModel *>(
                sensor_config.sensor);

            if (!distance_model || !distance_model->is_valid())
                continue;

            // Get actual reading
            units::Length measured = distance_model->get_measurement();
            float recorded = static_cast<float>(measured.to_inches());

            // Validate range
            if (recorded <= 0.0f || recorded > static_cast<float>(
                                                   units::Length::from_mm(2000.0).to_inches()))
                continue;

            // Compute sensor's world-frame pose using FieldMap (outside loop - fine)
            math::SE2 sensor_pose = field_map_.compute_sensor_global_pose(
                robot_pose,
                sensor_config.offset_forward,
                sensor_config.offset_lateral,
                sensor_config.bearing);

            // Get predicted distance for outlier rejection
            units::Length predicted = field_map_.compute_expected_distance(
                robot_pose,
                sensor_config.offset_forward,
                sensor_config.offset_lateral,
                sensor_config.bearing);

            if (predicted.to_inches() < 0.0)
                continue;

            // Simple threshold outlier rejection - extensible to Mahalanobis later
            float innovation = std::abs(recorded - static_cast<float>(predicted.to_inches()));
            if (innovation > pf_config_.sensor_tolerance)
                continue;

            // Compute std_dev from VEX sensor specs - same formula as guide
            float d = recorded;
            float bound = d < 7.874015f ? 0.590551f : 0.05f * d;
            float std_dev = std::max(bound / 3.0f, 1e-6f);

            // Precompute inv_var - done once here not per particle
            float inv_var = -0.5f / (std_dev * std_dev);

            // Extract sensor world position and ray direction as floats
            float rel_x = static_cast<float>(sensor_pose.x());
            float rel_y = static_cast<float>(sensor_pose.y());
            float ray_cos = static_cast<float>(std::cos(sensor_pose.theta()));
            float ray_sin = static_cast<float>(std::sin(sensor_pose.theta()));

            readings.push_back({recorded, inv_var, rel_x, rel_y, ray_cos, ray_sin});
        }

        // If no valid readings, skip weight update
        if (readings.empty())
            return;

        // Hoist wall bounds - never changes
        const auto bounds = field_map_.get_wall_bounds_f32();

        // Weight each particle - pure float from here
        float max_weight = 0.0f;

        const size_t N = pf_config_.num_particles;

        for (size_t i = 0; i < N; i++)
        {
            float weight = 1.0f;

            for (const auto &r : readings)
            {
                // Sensor world position offset by particle position
                float sx = r.rel_x + particle_x_[i];
                float sy = r.rel_y + particle_y_[i];

                float predicted = math::raycast_aabb(sx, sy, r.ray_cos, r.ray_sin, bounds);

                if (predicted < 0.0f)
                {
                    weight = 0.0f;
                    break;
                }

                float error = r.recorded - predicted;
                weight *= std::exp(r.inv_var * error * error);

                if (weight == 0.0f)
                    break;
            }

            if (!std::isfinite(weight) || weight < 0.0f)
                weight = 0.0f;

            particle_weights_[i] = weight;
            if (weight > max_weight)
                max_weight = weight;
        }

        // Normalize weights
        if (max_weight <= 0.0f)
        {
            // All weights zero - reset to uniform
            const float uniform = 1.0f / static_cast<float>(N);
            std::fill(particle_weights_.begin(), particle_weights_.end(), uniform);
            return;
        }

        float weight_sum = 0.0f;
        for (size_t i = 0; i < N; i++)
        {
            particle_weights_[i] /= max_weight;
            weight_sum += particle_weights_[i];
        }

        if (weight_sum <= 0.0f)
        {
            const float uniform = 1.0f / static_cast<float>(N);
            std::fill(particle_weights_.begin(), particle_weights_.end(), uniform);
            return;
        }

        float inv_weight_sum = 1.0f / weight_sum;
        for (size_t i = 0; i < N; i++)
        {
            particle_weights_[i] *= inv_weight_sum;
        }

        // Update pose estimate then resample
        {
            std::lock_guard<pros::Mutex> lock(pose_mutex_);
            Pose estimated = estimate();
            current_pose_.se2 = estimated.se2;
        }

        resample();
    }

    Pose ParticleFilterEstimator::estimate() const
    {
        float est_x = 0.0f;
        float est_y = 0.0f;

        const size_t N = pf_config_.num_particles;

        for (size_t i = 0; i < N; i++)
        {
            est_x += particle_x_[i] * particle_weights_[i];
            est_y += particle_y_[i] * particle_weights_[i];
        }

        // Keep current heading - IMU is more accurate than particles for theta
        Pose result;
        result.se2 = math::SE2(
            static_cast<double>(est_x),
            static_cast<double>(est_y),
            current_pose_.theta_rad());

        return result;
    }

    void ParticleFilterEstimator::resample()
    {
        const size_t N = pf_config_.num_particles;

        // Save presample state for debugging before we overwrite
        if (pf_config_.keep_presample_arrays)
        {
            std::copy(particle_x_.begin(), particle_x_.end(), presample_x_.begin());
            std::copy(particle_y_.begin(), particle_y_.end(), presample_y_.begin());
            std::copy(particle_weights_.begin(), particle_weights_.end(), presample_weights_.begin());
        }

        // Systematic resampling
        const float inv_n = 1.0f / static_cast<float>(N);

        // Single random offset for the whole sweep - this is what makes it
        // systematic rather than multinomial, O(N) not O(N^2)
        const float offset = rng_.next_f32() * inv_n;

        float cumulative_weight = particle_weights_[0];
        size_t idx = 0;

        for (size_t i = 0; i < N; i++)
        {
            float sample = offset + static_cast<float>(i) * inv_n;

            // Advance idx until cumulative weight covers this sample
            // Note: idx is NOT reset each iteration - that's what makes this O(N)
            while (sample > cumulative_weight && idx < N - 1)
            {
                idx++;
                cumulative_weight += particle_weights_[idx];
            }

            temp_x_[i] = particle_x_[idx];
            temp_y_[i] = particle_y_[idx];
            temp_weights_[i] = inv_n;
        }

        // Swap temp into main arrays
        std::copy(temp_x_.begin(), temp_x_.end(), particle_x_.begin());
        std::copy(temp_y_.begin(), temp_y_.end(), particle_y_.begin());
        std::copy(temp_weights_.begin(), temp_weights_.end(), particle_weights_.begin());
    }
}