#include "geometric_odometry_estimator.hpp"
#include <cmath>
#include <mutex>
#include "abclib/math/angles.hpp"
#include "abclib/telemetry/telemetry.hpp"

namespace abclib::estimation
{
    GeometricOdometryEstimator::GeometricOdometryEstimator(
        IMeasurementModel<units::Distance> *vertical_model,
        IMeasurementModel<units::Distance> *horizontal_model,
        IMeasurementModel<units::Radians> *imu_model,
        units::Distance vertical_offset,
        units::Distance horizontal_offset)
        : vertical_model_(vertical_model),
          horizontal_model_(horizontal_model),
          imu_model_(imu_model),
          vertical_offset_(vertical_offset),
          horizontal_offset_(horizontal_offset)
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

    void GeometricOdometryEstimator::update()
    {
        units::Distance delta_vertical = vertical_model_ ? vertical_model_->get_measurement() : units::Distance::from_inches(0.0);
        units::Distance delta_horizontal = horizontal_model_ ? horizontal_model_->get_measurement() : units::Distance::from_inches(0.0);
        units::Radians delta_imu = imu_model_ ? imu_model_->get_measurement() : units::Radians(0.0);

        LocalMotion local_motion = ArcLengthDifferentialDrive::compute_local_motion(
            delta_vertical,
            delta_horizontal,
            delta_imu,
            vertical_offset_,
            horizontal_offset_);

        double heading = current_pose_.theta() + delta_imu.value;
        double avg_heading = current_pose_.theta() + delta_imu.value / 2.0;

        static double prev_x = 0.0;
        static double prev_y = 0.0;
        static double prev_theta = 0.0;
        static bool first_update = true;

        std::lock_guard<pros::Mutex> lock(pose_mutex_);

        current_pose_.set_x(current_pose_.x() +
                            local_motion.y.inches * std::cos(avg_heading) +
                            local_motion.x.inches * std::sin(avg_heading));
        current_pose_.set_y(current_pose_.y() +
                            local_motion.y.inches * std::sin(avg_heading) -
                            local_motion.x.inches * std::cos(avg_heading));
        current_pose_.set_theta(heading);

        if (!first_update)
        {
            const double dt = 0.01;

            double dx = current_pose_.x() - prev_x;
            double dy = current_pose_.y() - prev_y;
            double dtheta = current_pose_.theta() - prev_theta;

            dtheta = math::normalize_angle(dtheta);

            double v_raw = (dx * std::cos(current_pose_.theta()) +
                            dy * std::sin(current_pose_.theta())) /
                           dt;
            double omega_raw = dtheta / dt;

            const double alpha = 0.3;
            current_pose_.v = units::BodyLinearVelocity(
                alpha * v_raw + (1.0 - alpha) * current_pose_.v.inches_per_sec);
            current_pose_.omega = units::BodyAngularVelocity(
                alpha * omega_raw + (1.0 - alpha) * current_pose_.omega.rad_per_sec);

            {
                std::lock_guard<pros::Mutex> telem_lock(abclib::telemetry_mutex);
                abclib::telemetry.pose = current_pose_.pose;
                abclib::telemetry.pose_v = current_pose_.v;
                abclib::telemetry.pose_omega = current_pose_.omega;
                abclib::telemetry.pose_v_raw = units::BodyLinearVelocity(v_raw);
                abclib::telemetry.pose_omega_raw = units::BodyAngularVelocity(omega_raw);
            }
        }
        else
        {
            current_pose_.v = units::BodyLinearVelocity(0.0);
            current_pose_.omega = units::BodyAngularVelocity(0.0);
            first_update = false;
        }

        prev_x = current_pose_.x();
        prev_y = current_pose_.y();
        prev_theta = current_pose_.theta();
    }

    void GeometricOdometryEstimator::set_pose(const Pose &pose)
    {
        std::lock_guard<pros::Mutex> lock(pose_mutex_);
        current_pose_ = pose;

        current_pose_.v = units::BodyLinearVelocity(0.0);
        current_pose_.omega = units::BodyAngularVelocity(0.0);
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