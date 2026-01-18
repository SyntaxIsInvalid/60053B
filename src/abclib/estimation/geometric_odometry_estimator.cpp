#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include <cmath>
#include <mutex>
#include "abclib/math/angles.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/estimation/distance_measurement_model.hpp"
#include "abclib/field/field_map.hpp"
namespace abclib::estimation
{
    GeometricOdometryEstimator::GeometricOdometryEstimator(
        IMeasurementModel<units::Length> *vertical_model,
        IMeasurementModel<units::Length> *horizontal_model,
        IMeasurementModel<units::Angle> *imu_model,
        units::Length vertical_offset,
        units::Length horizontal_offset,
        const field::FieldConfig &field_config,
        IMeasurementModel<units::Length> *distance_sensor)
        : vertical_model_(vertical_model),
          horizontal_model_(horizontal_model),
          imu_model_(imu_model),
          distance_sensor_(distance_sensor),
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

    void GeometricOdometryEstimator::apply_distance_correction()
    {
        // Early exit if correction is disabled or no sensor
        if (!distance_correction_enabled_ || !distance_sensor_)
        {
            return;
        }

        // Get measurement from sensor
        units::Length measured_distance = distance_sensor_->get_measurement();

        // Check if reading is valid
        auto *distance_model = dynamic_cast<DistanceSensorMeasurementModel *>(distance_sensor_);
        if (!distance_model || !distance_model->is_valid())
        {
            return;
        }

        // Get sensor position in field frame
        double sensor_x_field, sensor_y_field;
        field_map_.compute_sensor_global_position(
            current_pose_.x_inches(),
            current_pose_.y_inches(),
            current_pose_.theta_rad(),
            sensor_offset_forward_.to_inches(),
            sensor_offset_lateral_.to_inches(),
            sensor_x_field,
            sensor_y_field);

        // Get which wall the sensor is facing
        field::FieldMap::Wall wall = field_map_.get_nearest_wall(
            current_pose_.x_inches(),
            current_pose_.y_inches(),
            current_pose_.theta_rad(),
            0.0);

        // Compute expected distance to that wall
        double expected_distance = field_map_.compute_distance_to_wall(
            sensor_x_field,
            sensor_y_field,
            current_pose_.theta_rad(),
            wall);

        // If ray doesn't hit wall (parallel case), skip correction
        if (expected_distance < 0.0)
        {
            return;
        }

        // Compute error
        double error_inches = measured_distance.to_inches() - expected_distance;

        // Compute correction vector in field frame
        double cos_theta = std::cos(current_pose_.theta_rad());
        double sin_theta = std::sin(current_pose_.theta_rad());

        double correction_x = -error_inches * cos_theta;
        double correction_y = -error_inches * sin_theta;

        // Apply blended correction
        current_pose_.set_x(current_pose_.x_inches() + distance_blend_factor_ * correction_x);
        current_pose_.set_y(current_pose_.y_inches() + distance_blend_factor_ * correction_y);
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

            telem.heading_wall = field_map_.get_nearest_wall(
                current_pose_.x_inches(),
                current_pose_.y_inches(),
                current_pose_.theta_rad(),
                0.0);

            double distance = field_map_.compute_distance_to_wall(
                current_pose_.x_inches(),
                current_pose_.y_inches(),
                current_pose_.theta_rad(),
                telem.heading_wall);

            telem.heading_wall_valid = (distance >= 0.0);
            telem.heading_distance_to_wall = telem.heading_wall_valid ? units::Length::from_inches(distance) : units::Length::from_inches(-1.0);

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