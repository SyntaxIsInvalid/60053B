#include "abclib/control/pure_pursuit.hpp"
#include <cmath>
#include "abclib/math/angles.hpp"
namespace abclib::control
{
    double PurePursuit::calculate_curvature(
        const path::Point &lookahead_point,
        const estimation::Pose &robot_pose,
        units::Length lookahead_distance)
    {
        // Transform lookahead point to robot frame
        units::Length dx = units::Length::from_inches(lookahead_point.x()) - robot_pose.x();
        units::Length dy = units::Length::from_inches(lookahead_point.y()) - robot_pose.y();

        units::Angle heading = robot_pose.theta();
        double cos_h = cos(heading);
        double sin_h = sin(heading);

        // Rotate to robot frame (robot at origin, facing +x)
        units::Length local_x = dx * cos_h + dy * sin_h;
        units::Length local_y = dx * (-sin_h) + dy * cos_h;

        // Pure pursuit curvature formula: κ = 2y / L²
        double curvature = 2.0 * local_y.to_inches() / (lookahead_distance.to_inches() * lookahead_distance.to_inches());

        return curvature;
    }

    void PurePursuit::curvature_to_wheel_velocities(
        double curvature,
        units::Velocity linear_velocity,
        units::Length track_width,
        units::Velocity &left_velocity,
        units::Velocity &right_velocity)
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

    double PurePursuit::calculate_heading_correction(
        units::Angle current_heading,
        units::Angle target_heading,
        double progress,
        const PurePursuitConfig &config)
    {
        if (!config.use_heading_correction)
        {
            return 0.0;
        }

        // Only apply correction when we're near the end
        if (progress < config.heading_start_threshold)
        {
            return 0.0;
        }

        // Normalize heading error to [-pi, pi]
        units::Angle heading_error = target_heading - current_heading;
        double error_rad = math::normalize_angle(heading_error.to_radians());

        // Calculate blend factor (0 at threshold, 1 at end)
        double blend_range = 1.0 - config.heading_start_threshold;
        double blend_factor = (progress - config.heading_start_threshold) / blend_range;
        blend_factor = std::clamp(blend_factor, 0.0, 1.0);

        // Proportional controller: angular_velocity [rad/s] = kP [1/s] * error [rad]
        double angular_velocity = error_rad * config.heading_correction_gain * blend_factor;

        return angular_velocity; // rad/s
    }

} // namespace abclib::control