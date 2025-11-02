#if 0
#pragma once
#include "api.h"
#include <optional>
#include "abclib/hardware/tracking_wheel_interface.hpp"
#include "abclib/units/units.hpp"

namespace abclib::estimation
{
    struct Pose
    {
        units::BodyPose pose;             // Combined position + heading
        units::BodyLinearVelocity v;      // linear velocity
        units::BodyAngularVelocity omega; // angular velocity

        Pose() = default;

        Pose(double x, double y, double theta_radians)
            : pose(units::BodyPose::from_radians(x, y, theta_radians)),
              v(units::BodyLinearVelocity(0)),
              omega(units::BodyAngularVelocity(0)) {}

        Pose(units::BodyPose p, units::BodyLinearVelocity vel, units::BodyAngularVelocity ang_vel)
            : pose(p), v(vel), omega(ang_vel) {}

        // Convenience accessors
        double x() const { return pose.x(); }
        double y() const { return pose.y(); }
        double theta() const { return pose.theta(); }
        units::Degrees theta_deg() const { return pose.theta_deg(); }
        void set_x(double x) { pose.position.x_inches = x; }
        void set_y(double y) { pose.position.y_inches = y; }
        void set_theta(double theta_rad) { pose.heading.angle.value = theta_rad; }
    };

    class Odometry
    {
    private:
        // Sensors
        hardware::ITrackingWheel *vertical;
        hardware::ITrackingWheel *horizontal;
        pros::IMU *imu;

        // State
        Pose current_pose{};
        double prev_vertical = 0.0;
        double prev_horizontal = 0.0;
        double prev_imu = 0.0;

        // Threading
        std::optional<pros::Task> tracking_task;
        mutable pros::Mutex task_mutex_;
        mutable pros::Mutex pose_mutex;

    public:
        Odometry(hardware::ITrackingWheel *vertical_wheel,
                 hardware::ITrackingWheel *horizontal_wheel,
                 pros::IMU *imu_sensor);
        ~Odometry();

        void init();
        void stop();

        // Reset sensors AND pose (full calibration)
        void reset();

        // Update pose WITHOUT resetting sensors (e.g., vision correction)
        void set_pose(const Pose &pose);

        // Getters
        Pose get_pose() const; // Returns full state

        void update();
        void calibrate_sensors(); // Reset sensors only, keep pose
    };
}
#endif