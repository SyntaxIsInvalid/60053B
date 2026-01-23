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

    units::Velocity PurePursuit::apply_curvature_constraint(
        units::Velocity raw_velocity,
        double curvature,
        units::Length min_radius)
    {
        // Straight line or near-straight - no constraint needed
        if (std::abs(curvature) < 1e-6)
        {
            return raw_velocity;
        }

        // Calculate radius of curvature
        double radius = std::abs(1.0 / curvature); // inches

        // Gentle curve - no constraint needed
        if (radius >= min_radius.to_inches())
        {
            return raw_velocity;
        }

        // Tight turn - apply linear velocity reduction
        // v_constrained = v_raw * (r / r_min)
        double scale_factor = radius / min_radius.to_inches();
        return units::Velocity::from_ips(raw_velocity.to_ips() * scale_factor);
    }

    double PurePursuit::calculate_rotation_velocity(
        units::Angle heading_error,
        units::AngularVelocity current_omega,
        const PurePursuitConfig &config,
        double dt)
    {
        // 1. Direction of rotation
        double error_rad = heading_error.to_radians();
        const double sign = error_rad > 0.0 ? 1.0 : -1.0;

        // 2. Desired angular velocity (max)
        double desired_omega = sign * config.rotate_to_heading_angular_vel.to_rad_per_sec();

        // 3. Respect kinematic acceleration limits
        double current_omega_rad = current_omega.to_rad_per_sec();
        double max_accel_rad = config.max_angular_accel.to_rad_per_sec2();

        double min_feasible_omega = current_omega_rad - max_accel_rad * dt;
        double max_feasible_omega = current_omega_rad + max_accel_rad * dt;
        desired_omega = std::clamp(desired_omega, min_feasible_omega, max_feasible_omega);

        // 4. Avoid overshooting (stopping distance constraint)
        // v² = 2aθ  →  v_max = sqrt(2 * a * |θ|)
        double max_vel_to_stop = std::sqrt(2.0 * max_accel_rad * std::abs(error_rad));
        if (std::abs(desired_omega) > max_vel_to_stop)
        {
            desired_omega = sign * max_vel_to_stop;
        }

        return desired_omega;
    }
} // namespace abclib::control