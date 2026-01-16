// chassis_ramsete.cpp (updated)
#include "abclib/hardware/chassis.hpp"
#include "abclib/control/ramsete.hpp"
#include "abclib/path/quintic_hermite_segment.hpp"
#include "abclib/trajectory/trajectory.hpp"
#include <algorithm>

namespace abclib::hardware
{
    void Chassis::follow_path_ramsete(
        const path::Path& path,
        units::Time timeout)
    {
        if (path.empty()) {
            return;
        }

        // Create RAMSETE controller with your tuned constants
        control::Ramsete ramsete(config_.controllers.ramsete);

        // Get the profile group and create trajectory
        const auto& group = path.get_profile_groups()[0];
        trajectory::Trajectory traj(&group);

        uint32_t start_time = pros::millis();
        units::Time total_time = traj.get_total_time();

        while ((pros::millis() - start_time) < timeout.to_milliseconds())
        {
            // Calculate elapsed time
            units::Time elapsed = units::Time::from_milliseconds(pros::millis() - start_time);

            // Check if trajectory is complete
            if (traj.is_complete(elapsed))
            {
                stop_motors();
                break;
            }

            // Get current robot state
            estimation::Pose robot_pose = get_pose();

            // Get reference state from trajectory
            trajectory::TrajectoryState ref = traj.get_state(elapsed);

            // Compute RAMSETE control law
            control::RamseteOutput output = ramsete.compute(robot_pose, ref);

            // Convert body velocities (v, omega) to wheel velocities
            // For differential drive: v_left = v - omega * track_width / 2
            //                        v_right = v + omega * track_width / 2
            double v_command = output.v.to_ips();
            double omega_command = output.omega.to_rad_per_sec();
            double half_track = track_width.to_inches() / 2.0;

            units::Velocity left_vel = units::Velocity::from_ips(
                v_command - omega_command * half_track);
            units::Velocity right_vel = units::Velocity::from_ips(
                v_command + omega_command * half_track);

            // Command motors
            move_velocity(left_vel, right_vel, 0, 0);

            // Update telemetry with tracking errors
            update_lateral_telemetry(
                output.e_x,
                v_command,  // Use forward velocity as "output"
                units::Length::from_inches(ref.x),
                robot_pose.x,
                0.01
            );

            update_angular_telemetry(
                output.e_theta.to_radians(),
                omega_command,
                ref.theta,
                robot_pose.theta_rad(),
                0.01
            );

            update_pose_telemetry(robot_pose);

            pros::delay(10);  // 100Hz control loop
        }

        stop_motors();
    }

    void Chassis::quintic_ramsete(
        units::Length target_x,
        units::Length target_y,
        units::Angle target_heading,
        units::Velocity max_velocity,
        units::Acceleration max_acceleration,
        units::Time timeout)
    {
        // Get current pose
        estimation::Pose current = get_pose();
        
        // Create start and end poses
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
        
        // Build quintic path with motion profile
        path::Path quintic_path;
        path::ProfileGroup group(
            "quintic_ramsete", 
            max_velocity,
            max_acceleration
        );
        
        auto segment = std::make_unique<path::QuinticHermiteSegment>(start, end);
        group.add_segment(std::move(segment));
        group.compute_arc_length();  // Builds the trapezoidal velocity profile
        
        quintic_path.add_profile_group(std::move(group));
        
        // Follow with RAMSETE
        follow_path_ramsete(quintic_path, timeout);
    }

} // namespace abclib::hardware