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
        // Convert lookahead point to Eigen vector
        Eigen::Vector2d global_point(lookahead_point.x(), lookahead_point.y());
        
        // Transform to robot's local frame
        Eigen::Vector2d local_point = robot_pose.global_to_local(global_point);
        
        // Pure pursuit curvature formula: κ = 2y / L²
        double curvature = 2.0 * local_point.y() / 
                          (lookahead_distance.to_inches() * lookahead_distance.to_inches());
        
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
        left_velocity = units::Velocity::from_ips(v * (1.0 - curvature * d / 2.0));
        right_velocity = units::Velocity::from_ips(v * (1.0 + curvature * d / 2.0));
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

        // Normalize heading error to [-π, π]
        units::Angle heading_error = target_heading - current_heading;
        double error_rad = math::normalize_angle(heading_error.to_radians());

        // Calculate blend factor (0 at threshold, 1 at end)
        double blend_range = 1.0 - config.heading_start_threshold;
        double blend_factor = (progress - config.heading_start_threshold) / blend_range;
        blend_factor = std::clamp(blend_factor, 0.0, 1.0);

        // Proportional controller: ω [rad/s] = kP [1/s] × error [rad]
        double angular_velocity = error_rad * config.heading_correction_gain * blend_factor;

        return angular_velocity;
    }

} // namespace abclib::control