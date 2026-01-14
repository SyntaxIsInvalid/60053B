// chassis_pure_pursuit.cpp
#include "abclib/hardware/chassis.hpp"
#include "abclib/control/pure_pursuit.hpp"
#include "abclib/path/path_segment_interface.hpp"
#include <algorithm>
#include <cmath>
#include "abclib/path/quintic_hermite_segment.hpp"

namespace abclib::hardware
{
    void Chassis::follow_path_pure_pursuit(
    const path::Path& path,
    const control::PurePursuitConfig& config,
    units::Time timeout)
{
    uint32_t start_time = pros::millis();
    double current_arc_length = 0.0;
    double total_length = path.get_total_arc_length();
    const units::Length end_threshold = units::Length::from_inches(2.0);

    while ((pros::millis() - start_time) < timeout.to_milliseconds())
    {
        estimation::Pose robot_pose = get_pose();

        // Check if we're at the end
        double distance_to_end = total_length - current_arc_length;
        if (distance_to_end < end_threshold.to_inches())
        {
            stop_motors();
            break;
        }

        // Find closest point on path
        double closest_s = find_closest_arc_length_on_path(path, robot_pose);
        current_arc_length = std::max(current_arc_length, closest_s);

        double progress = current_arc_length / total_length;

        // NEW: Get velocity from profile or config
        units::Velocity target_velocity;
        if (config.use_motion_profile) {
            const auto& group = path.get_profile_groups()[0];  // Single group for quintic
            target_velocity = group.get_velocity_at_arc_length(current_arc_length);
        } else {
            target_velocity = config.target_velocity;
        }

        // Calculate lookahead distance
        units::Length current_lookahead;
        if (config.use_adaptive_lookahead) {
            double current_speed = robot_pose.v.to_ips();
            double adaptive_lookahead = current_speed * config.lookahead_time.to_seconds();
            current_lookahead = units::Length::from_inches(
                std::clamp(adaptive_lookahead,
                          config.min_lookahead.to_inches(),
                          config.max_lookahead.to_inches())
            );
        } else {
            current_lookahead = config.lookahead_distance;
        }

        // Calculate lookahead point
        double lookahead_s = current_arc_length + current_lookahead.to_inches();
        lookahead_s = std::min(lookahead_s, total_length);
        path::Point lookahead_point = path.get_point_at_arc_length(lookahead_s);

        // Calculate curvature
        double curvature = control::PurePursuit::calculate_curvature(
            lookahead_point,
            robot_pose,
            current_lookahead
        );

        const auto& group = path.get_profile_groups()[0];
        double target_heading = group.query_heading(lookahead_s);
        
        // Calculate heading correction
        double heading_correction_rad_per_s = control::PurePursuit::calculate_heading_correction(
            robot_pose.theta.to_radians(),
            target_heading,
            progress,
            config
        );


        // Convert to wheel velocities
        units::Velocity left_vel, right_vel;
        control::PurePursuit::curvature_to_wheel_velocities(
            curvature,
            target_velocity,  // Now uses profiled velocity when enabled
            track_width,
            left_vel,
            right_vel
        );

        if (config.use_heading_correction) {
            // Convert angular velocity [rad/s] to differential wheel velocity [in/s]
            // \omega = (v_right - v_left) / track_width → \delta v = ω * track_width / 2
            double correction_ips = heading_correction_rad_per_s * track_width.to_inches() / 2.0;
            
            left_vel = units::Velocity::from_ips(left_vel.to_ips() - correction_ips);
            right_vel = units::Velocity::from_ips(right_vel.to_ips() + correction_ips);
        }

        // Command motors
        move_velocity(left_vel, right_vel);

        pros::delay(10);
    }

    stop_motors();
    if (config.use_final_turn) {
        estimation::Pose final_pose = get_pose();
        const auto& group = path.get_profile_groups()[0];
        path::Pose target_pose = group.get_end_pose();
        double target_heading_rad = target_pose(2);
        
        // Calculate final heading error
        double heading_error = target_heading_rad - final_pose.theta.to_radians();
        while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
        
        // Check if we need to correct
        if (std::abs(heading_error) > config.final_heading_tolerance.to_radians()) {
            turn_to_heading(
                units::Angle::from_radians(target_heading_rad),
                config.final_turn_timeout
            );
        }
    }
}


    double Chassis::find_closest_arc_length_on_path(
        const path::Path& path, 
        const estimation::Pose& robot_pose) const
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

            double dx = path_point.x() - robot_pose.x.to_inches();
            double dy = path_point.y() - robot_pose.y.to_inches();
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
            
            double dx = path_point.x() - robot_pose.x.to_inches();
            double dy = path_point.y() - robot_pose.y.to_inches();
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
    const control::PurePursuitConfig& config,
    units::Time timeout)
{
    estimation::Pose current = get_pose();
    
    path::Pose start(
        current.x.to_inches(),
        current.y.to_inches(),
        current.theta.to_radians()
    );
    
    path::Pose end(
        target_x.to_inches(),
        target_y.to_inches(),
        target_heading.to_radians()
    );
    
    path::Path quintic_path;
    path::ProfileGroup group(
        "quintic_move", 
        config.target_velocity,      // Use from config
        config.max_acceleration      // Use from config
    );
    
    auto segment = std::make_unique<path::QuinticHermiteSegment>(start, end);
    group.add_segment(std::move(segment));
    group.compute_arc_length();  // This builds the motion profile
    
    quintic_path.add_profile_group(std::move(group));
    
    follow_path_pure_pursuit(quintic_path, config, timeout);
}



} // namespace abclib::hardware