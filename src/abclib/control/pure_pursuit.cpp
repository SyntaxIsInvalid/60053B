#include "abclib/control/pure_pursuit.hpp"
#include <cmath>

namespace abclib::control
{
    double PurePursuit::calculate_curvature(
        const path::Point& lookahead_point,
        const estimation::Pose& robot_pose,
        units::Length lookahead_distance)
    {
        // Transform lookahead point to robot frame
        double dx = lookahead_point.x() - robot_pose.x.to_inches();
        double dy = lookahead_point.y() - robot_pose.y.to_inches();

        double heading = robot_pose.theta.to_radians();
        double cos_h = std::cos(heading);
        double sin_h = std::sin(heading);

        // Rotate to robot frame (robot at origin, facing +x)
        double local_x = cos_h * dx + sin_h * dy;
        double local_y = -sin_h * dx + cos_h * dy;

        // Pure pursuit curvature formula: κ = 2y / L²
        double L = lookahead_distance.to_inches();
        double curvature = 2.0 * local_y / (L * L);

        return curvature;
    }

    void PurePursuit::curvature_to_wheel_velocities(
        double curvature,
        units::Velocity linear_velocity,
        units::Length track_width,
        units::Velocity& left_velocity,
        units::Velocity& right_velocity)
    {
        double v = linear_velocity.to_ips();
        double d = track_width.to_inches();

        // Differential drive kinematics with curvature
        // v_left = v(1 - κd/2)
        // v_right = v(1 + κd/2)
        double left = v * (1.0 - curvature * d / 2.0);
        double right = v * (1.0 + curvature * d / 2.0);

        left_velocity = units::Velocity::from_ips(left);
        right_velocity = units::Velocity::from_ips(right);
    }

} // namespace abclib::control