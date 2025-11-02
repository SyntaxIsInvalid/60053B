#pragma once
#include "abclib/units/units.hpp"

namespace abclib::estimation
{
    struct Pose
    {
        units::BodyPose pose;
        units::BodyLinearVelocity v;
        units::BodyAngularVelocity omega;

        Pose() = default;

        Pose(double x, double y, double theta_radians)
            : pose(units::BodyPose::from_radians(x, y, theta_radians)),
              v(units::BodyLinearVelocity(0)),
              omega(units::BodyAngularVelocity(0)) {}

        Pose(units::BodyPose p, units::BodyLinearVelocity vel, units::BodyAngularVelocity ang_vel)
            : pose(p), v(vel), omega(ang_vel) {}

        double x() const { return pose.x(); }
        double y() const { return pose.y(); }
        double theta() const { return pose.theta(); }
        units::Degrees theta_deg() const { return pose.theta_deg(); }
        void set_x(double x) { pose.position.x_inches = x; }
        void set_y(double y) { pose.position.y_inches = y; }
        void set_theta(double theta_rad) { pose.heading.angle.value = theta_rad; }
    };
}