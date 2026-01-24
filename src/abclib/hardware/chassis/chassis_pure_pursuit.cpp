#include "abclib/hardware/chassis.hpp"
#include "abclib/control/pure_pursuit.hpp"
#include "abclib/path/path_segment_interface.hpp"
#include <algorithm>
#include <cmath>
#include "abclib/path/quintic_hermite_segment.hpp"
#include "abclib/telemetry/path_logger.hpp"

namespace abclib::hardware
{
    void Chassis::follow_path_pure_pursuit(
        const path::Path &path,
        const control::PurePursuitConfig &config,
        units::Time timeout)
    {
        uint32_t start_time = pros::millis();
        double current_arc_length = 0.0;
        double total_length = path.get_total_arc_length();
        const double dt = 0.01; // 10ms loop

        // Reset stateful tracking
        has_reached_xy_goal_ = false;

        while ((pros::millis() - start_time) < timeout.to_milliseconds())
        {
            estimation::Pose robot_pose = get_pose_standard();
            const auto &group = path.get_profile_groups()[0];

            // Find closest point on path
            double closest_s = find_closest_arc_length_on_path(path, robot_pose);
            current_arc_length = std::max(current_arc_length, closest_s);

            // Calculate distance to end
            double distance_to_end = total_length - current_arc_length;

            // ========================================
            // INTEGRATED FINAL TURN LOGIC
            // ========================================
            bool should_rotate_to_goal = false;

            if (config.use_final_turn)
            {
                if (config.use_stateful_rotation)
                {
                    // Stateful mode: once triggered, stay in rotation mode
                    if (!has_reached_xy_goal_ && distance_to_end < config.rotation_distance_threshold.to_inches())
                    {
                        has_reached_xy_goal_ = true; // Latch it
                    }
                    should_rotate_to_goal = has_reached_xy_goal_;
                }
                else
                {
                    // Non-stateful: re-evaluate every iteration
                    should_rotate_to_goal = (distance_to_end < config.rotation_distance_threshold.to_inches());
                }
            }

            // Check if rotation is complete
            if (should_rotate_to_goal)
            {
                path::Pose target_pose = group.get_end_pose();
                units::Angle target_heading = units::Angle::from_radians(target_pose(2));
                units::Angle heading_error = target_heading - robot_pose.theta();

                // Normalize to [-π, π]
                double error_rad = heading_error.to_radians();
                while (error_rad > M_PI)
                    error_rad -= 2.0 * M_PI;
                while (error_rad < -M_PI)
                    error_rad += 2.0 * M_PI;
                heading_error = units::Angle::from_radians(error_rad);

                // Check if we're done
                if (std::abs(heading_error.to_radians()) < config.final_heading_tolerance.to_radians())
                {
                    stop_motors();
                    break; // Goal achieved!
                }

                // ========================================
                // ROTATE TO GOAL HEADING
                // ========================================
                double angular_vel_rad = control::PurePursuit::calculate_rotation_velocity(
                    heading_error,
                    robot_pose.omega,
                    config,
                    dt);

                // Convert to wheel velocities (pure rotation: opposite directions)
                double wheel_vel_ips = angular_vel_rad * track_width.to_inches() / 2.0;
                units::Velocity left_vel = units::Velocity::from_ips(-wheel_vel_ips);
                units::Velocity right_vel = units::Velocity::from_ips(wheel_vel_ips);

                move_velocity(left_vel, right_vel);
                pros::delay(10);
                continue; // Skip normal pure pursuit this iteration
            }

            if (!config.use_final_turn && distance_to_end < 1)
            {
                path::Pose end_pose = group.get_end_pose();
                double dx = end_pose(0) - robot_pose.x().to_inches();
                double dy = end_pose(1) - robot_pose.y().to_inches();
                double distance_to_goal = std::sqrt(dx * dx + dy * dy);

                if (distance_to_goal < settlement_config_.position_threshold.to_inches())
                {
                    stop_motors();
                    break; // Exit - reached goal!
                }
            }

            // ========================================
            // NORMAL PURE PURSUIT TRACKING
            // ========================================

            // Get velocity from profile
            units::Velocity target_velocity;
            if (config.use_motion_profile)
            {
                target_velocity = group.get_velocity_at_arc_length(current_arc_length);
            }
            else
            {
                target_velocity = config.target_velocity;
            }

            // Calculate lookahead distance
            units::Length current_lookahead;
            if (config.use_adaptive_lookahead)
            {
                double current_speed = robot_pose.v.to_ips();
                double adaptive_lookahead = current_speed * config.lookahead_time.to_seconds();
                current_lookahead = units::Length::from_inches(
                    std::clamp(adaptive_lookahead,
                               config.min_lookahead.to_inches(),
                               config.max_lookahead.to_inches()));
            }
            else
            {
                current_lookahead = config.max_lookahead;
            }

            // Calculate lookahead point
            double lookahead_s = current_arc_length + current_lookahead.to_inches();
            lookahead_s = std::min(lookahead_s, total_length);
            path::Point lookahead_point = path.get_point_at_arc_length(lookahead_s);

            // Apply curvature regulation
            if (config.use_curvature_regulation)
            {
                double path_curvature = group.query_curvature(lookahead_s);
                target_velocity = control::PurePursuit::apply_curvature_constraint(
                    target_velocity,
                    path_curvature,
                    config.min_radius);
            }

            // Calculate control curvature for steering
            double control_curvature = control::PurePursuit::calculate_curvature(
                lookahead_point,
                robot_pose,
                current_lookahead);

            // Get target heading for heading correction
            double progress = current_arc_length / total_length;
            double target_heading = group.query_heading(lookahead_s);
            double heading_correction_rad_per_s = control::PurePursuit::calculate_heading_correction(
                robot_pose.theta(),
                units::Angle::from_radians(target_heading),
                progress,
                config);

            // Convert to wheel velocities
            units::Velocity left_vel, right_vel;
            control::PurePursuit::curvature_to_wheel_velocities(
                control_curvature,
                target_velocity,
                track_width,
                left_vel,
                right_vel);

            // Apply heading correction
            if (config.use_heading_correction)
            {
                double correction_ips = heading_correction_rad_per_s * track_width.to_inches() / 2.0;
                left_vel = units::Velocity::from_ips(left_vel.to_ips() - correction_ips);
                right_vel = units::Velocity::from_ips(right_vel.to_ips() + correction_ips);
            }

            // Command motors
            move_velocity(left_vel, right_vel);
            pros::delay(10);
        }

        stop_motors();
    }

    double Chassis::find_closest_arc_length_on_path(
        const path::Path &path,
        const estimation::Pose &robot_pose) const
    {
        // Simple brute-force search
        double min_distance_sq = std::numeric_limits<double>::max();
        double closest_s = 0.0;

        const double total_length = path.get_total_arc_length();
        const int num_samples = 50; // Check 50 points along the path

        // Coarse search
        for (int i = 0; i <= num_samples; ++i)
        {
            double s = (static_cast<double>(i) / num_samples) * total_length;
            path::Point path_point = path.get_point_at_arc_length(s);

            double dx = path_point.x() - robot_pose.x().to_inches();
            double dy = path_point.y() - robot_pose.y().to_inches();
            double distance_sq = dx * dx + dy * dy;

            if (distance_sq < min_distance_sq)
            {
                min_distance_sq = distance_sq;
                closest_s = s;
            }
        }

        // Refinement: check nearby points for better accuracy
        const double search_range = total_length / num_samples;
        const int refine_samples = 10;

        for (int i = -refine_samples; i <= refine_samples; ++i)
        {
            double s = closest_s + (static_cast<double>(i) / refine_samples) * search_range;
            s = std::clamp(s, 0.0, total_length);

            path::Point path_point = path.get_point_at_arc_length(s);

            double dx = path_point.x() - robot_pose.x().to_inches();
            double dy = path_point.y() - robot_pose.y().to_inches();
            double distance_sq = dx * dx + dy * dy;

            if (distance_sq < min_distance_sq)
            {
                min_distance_sq = distance_sq;
                closest_s = s;
            }
        }

        return closest_s;
    }

    void Chassis::quintic_pure_pursuit(
        units::Length target_x,
        units::Length target_y,
        units::Angle target_heading,
        const control::PurePursuitConfig &config,
        units::Time timeout)
    {
        // Reset state before starting new path
        reset_pure_pursuit_state();

        // Get current pose in STANDARD frame (CHANGED)
        estimation::Pose current_standard = get_pose_standard();

        // Convert target from CORNER → STANDARD (NEW)
        estimation::Pose target_corner(
            target_x, target_y, target_heading,
            units::Velocity::from_ips(0),
            units::AngularVelocity::from_rad_per_sec(0));

        estimation::Pose target_standard = field::alliance_corner_to_standard(
            target_corner, alliance_, config_.field_config);

        // Create path poses in STANDARD frame (CHANGED)
        path::Pose start(
            current_standard.x().to_inches(),
            current_standard.y().to_inches(),
            current_standard.theta().to_radians());

        path::Pose end(
            target_standard.x().to_inches(),
            target_standard.y().to_inches(),
            target_standard.theta().to_radians());

        // Rest remains the same
        path::Path quintic_path;
        path::ProfileGroup group(
            "pure_pursuit_quintic",
            config.target_velocity,
            config.max_acceleration);

        auto segment = std::make_unique<path::QuinticHermiteSegment>(start, end);
        group.add_segment(std::move(segment));
        group.compute_arc_length();

        quintic_path.add_profile_group(std::move(group));

        follow_path_pure_pursuit(quintic_path, config, timeout);
    }

} // namespace abclib::hardware