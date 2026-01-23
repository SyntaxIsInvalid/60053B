#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include <cmath>
#include <mutex>
#include "abclib/math/angles.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/measurement/distance_measurement_model.hpp"
#include "abclib/field/field_map.hpp"
#include "abclib/field/coordinate_transform.hpp"
namespace abclib::estimation
{
    GeometricOdometryEstimator::GeometricOdometryEstimator(
        IMeasurementModel<units::Length> *vertical_model,
        IMeasurementModel<units::Length> *horizontal_model,
        IMeasurementModel<units::Angle> *imu_model,
        units::Length vertical_offset,
        units::Length horizontal_offset,
        const field::FieldConfig &field_config,
        const std::vector<DistanceSensorConfig> &distance_sensors)
        : vertical_model_(vertical_model),
          horizontal_model_(horizontal_model),
          imu_model_(imu_model),
          distance_sensors_(distance_sensors), // Store the sensor array
          vertical_offset_(vertical_offset),
          horizontal_offset_(horizontal_offset),
          field_map_(field_config)
    {
    }

    void GeometricOdometryEstimator::init()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (!tracking_task_.has_value())
        {
            tracking_task_ = pros::Task([this]
                                        {
                while (pros::Task::notify_take(true, 0) == 0) {
                    this->update();
                    pros::delay(10);
                } });
        }
    }

    void GeometricOdometryEstimator::stop()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task_.has_value())
        {
            tracking_task_->notify();
            tracking_task_ = std::nullopt;
        }
    }

    GeometricOdometryEstimator::~GeometricOdometryEstimator()
    {
        std::lock_guard<pros::Mutex> lock(task_mutex_);
        if (tracking_task_.has_value())
        {
            tracking_task_->notify();
        }
    }

    void GeometricOdometryEstimator::reset()
    {
        current_pose_ = Pose();

        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();
    }

    Pose GeometricOdometryEstimator::get_pose() const
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        return current_pose_;
    }

    void GeometricOdometryEstimator::enable_sensor(size_t index, bool enable)
    {
        if (index < distance_sensors_.size())
        {
            distance_sensors_[index].enabled = enable;
        }
    }

    void GeometricOdometryEstimator::set_sensor_blend_factor(size_t index, double factor)
    {
        if (index < distance_sensors_.size())
        {
            distance_sensors_[index].blend_factor = factor;
        }
    }

    void GeometricOdometryEstimator::set_sensor_config(
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

    // geometric_odometry_estimator.cpp

    void GeometricOdometryEstimator::apply_distance_correction()
    {
        // Early exit if correction is disabled
        if (!distance_correction_enabled_)
        {
            return;
        }

        // Early exit if no sensors configured
        if (distance_sensors_.empty())
        {
            return;
        }

        // Accumulate weighted corrections from all sensors
        Eigen::Vector2d total_correction(0.0, 0.0);
        double total_weight = 0.0;

        // Loop through all sensors
        for (const auto &sensor_config : distance_sensors_)
        {
            // Skip if sensor is disabled or null
            if (!sensor_config.enabled || !sensor_config.sensor)
            {
                continue;
            }

            // Get measurement from sensor
            units::Length measured_distance = sensor_config.sensor->get_measurement();

            // Check if reading is valid
            auto *distance_model = dynamic_cast<DistanceSensorMeasurementModel *>(sensor_config.sensor);
            if (!distance_model || !distance_model->is_valid())
            {
                continue;
            }

            // Compute sensor's global pose using SE2 composition
            math::SE2 sensor_pose = field_map_.compute_sensor_global_pose(
                current_pose_,
                sensor_config.offset_x,
                sensor_config.offset_y,
                sensor_config.bearing);

            // Get which wall the sensor is facing
            field::FieldMap::Wall wall = field_map_.get_nearest_wall(
                current_pose_,
                sensor_config.bearing);

            // Compute expected distance to that wall
            units::Length expected_distance = field_map_.compute_distance_to_wall(
                sensor_pose,
                wall);

            // If ray doesn't hit wall (parallel case), skip this sensor
            if (expected_distance.to_inches() < 0.0)
            {
                continue;
            }

            // Compute error (positive = measured > expected = robot further from wall)
            double error_inches = measured_distance.to_inches() - expected_distance.to_inches();

            // Compute correction vector in field frame using sensor's heading
            double sensor_heading = sensor_pose.theta();
            double cos_theta = std::cos(sensor_heading);
            double sin_theta = std::sin(sensor_heading);

            // Correction points in sensor direction (negative error to correct closer)
            double correction_x = -error_inches * cos_theta;
            double correction_y = -error_inches * sin_theta;

            // Accumulate weighted correction
            total_correction.x() += correction_x * sensor_config.blend_factor;
            total_correction.y() += correction_y * sensor_config.blend_factor;
            total_weight += sensor_config.blend_factor;
        }

        // Apply the total weighted correction if any valid sensors contributed
        if (total_weight > 0.0)
        {
            // Average the weighted corrections
            double avg_correction_x = total_correction.x() / total_weight;
            double avg_correction_y = total_correction.y() / total_weight;

            // Apply blended correction to pose
            current_pose_.set_x(current_pose_.x_inches() + avg_correction_x);
            current_pose_.set_y(current_pose_.y_inches() + avg_correction_y);
        }
    }

    void GeometricOdometryEstimator::update()
    {
        // Get sensor deltas
        units::Length delta_vertical = vertical_model_ ? vertical_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Length delta_horizontal = horizontal_model_ ? horizontal_model_->get_measurement() : units::Length::from_inches(0.0);
        units::Angle delta_imu = imu_model_ ? imu_model_->get_measurement() : units::Angle::from_radians(0.0);

        // Compute local transformation (body frame)
        math::SE2 local_transform = ArcLengthDifferentialDrive::compute_local_transformation(
            delta_vertical,
            delta_horizontal,
            delta_imu,
            vertical_offset_,
            horizontal_offset_);

        std::lock_guard<pros::Mutex> lock(pose_mutex_);

        // Update pose: new_pose = old_pose * local_transform
        current_pose_.se2 = current_pose_.se2 * local_transform;

        // Apply distance sensor correction if enabled
        apply_distance_correction();

        // Compute velocities using the logarithm map
        const double dt = 0.01; // 10ms update rate

        // Get the tangent vector (body-frame velocity integrated over dt)
        Eigen::Vector3d xi = local_transform.log(); // [v_x, v_y, omega]^T

        // Extract velocity components
        double v_y_body = xi(1) / dt;  // Forward velocity (body frame)
        double omega_raw = xi(2) / dt; // Angular velocity

        // For non-holonomic robot, use forward velocity only
        double v_raw = v_y_body;

        // Exponential moving average filter
        const double alpha = 0.3;
        current_pose_.v = units::Velocity::from_ips(
            alpha * v_raw + (1.0 - alpha) * current_pose_.v.to_ips());
        current_pose_.omega = units::AngularVelocity::from_rad_per_sec(
            alpha * omega_raw + (1.0 - alpha) * current_pose_.omega.to_rad_per_sec());

        // Telemetry
        {
            auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
            telem.pose_standard = current_pose_;
            telem.pose_corner = field::standard_to_alliance_corner(
                current_pose_,
                telem.current_alliance, // Make sure this is set!
                field_map_.get_field_config());

            telem.heading_wall = field_map_.get_nearest_wall(
                current_pose_,
                units::Angle::from_radians(0.0));

            // Compute sensor pose for distance calculation
            math::SE2 sensor_pose = field_map_.compute_sensor_global_pose(
                current_pose_,
                units::Length::from_inches(0.0), // Assuming robot center for telemetry
                units::Length::from_inches(0.0),
                units::Angle::from_radians(0.0));

            units::Length distance = field_map_.compute_distance_to_wall(
                sensor_pose,
                telem.heading_wall);

            telem.heading_wall_valid = (distance.to_inches() >= 0.0);
            telem.heading_distance_to_wall = telem.heading_wall_valid ? distance : units::Length::from_inches(-1.0);

            telem.pose_v_raw = units::Velocity::from_ips(v_raw);
            telem.pose_omega_raw = units::AngularVelocity::from_rad_per_sec(omega_raw);

            abclib::telemetry::g_telemetry.swap();
        }
    }
    void GeometricOdometryEstimator::set_pose(const Pose &pose)
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        current_pose_ = pose;

        current_pose_.v = units::Velocity::from_ips(0.0);
        current_pose_.omega = units::AngularVelocity::from_rad_per_sec(0.0);
    }

    void GeometricOdometryEstimator::calibrate()
    {
        if (vertical_model_)
            vertical_model_->reset();
        if (horizontal_model_)
            horizontal_model_->reset();
        if (imu_model_)
            imu_model_->reset();

        current_pose_ = Pose();
    }

}