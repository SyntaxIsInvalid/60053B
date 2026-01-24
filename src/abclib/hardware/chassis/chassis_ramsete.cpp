#include "abclib/hardware/chassis.hpp"
#include "abclib/control/ramsete.hpp"
#include "abclib/path/path_segment_interface.hpp"
#include "abclib/path/quintic_hermite_segment.hpp"
#include <algorithm>
#include <cmath>

namespace abclib::hardware
{
    void Chassis::follow_path_ramsete(
        const path::Path &path,
        units::Time timeout)
    {
        uint32_t start_time = pros::millis();
        const auto &group = path.get_profile_groups()[0];
        units::Time total_time = group.get_total_time();

        control::Ramsete controller(config_.controllers.ramsete);

        while ((pros::millis() - start_time) < timeout.to_milliseconds())
        {
            double elapsed_seconds = (pros::millis() - start_time) / 1000.0;
            units::Time current_time = units::Time::from_seconds(elapsed_seconds);

            if (current_time >= total_time)
            {
                stop_motors();
                break;
            }

            // Get current pose in STANDARD frame
            estimation::Pose robot_pose = get_pose_standard();

            // Query trajectory state at current time
            path::Pose target_pose = group.query_at_time(current_time);
            units::Velocity v_ref = group.get_velocity_at_time(current_time);

            // Compute reference angular velocity: ω = v·κ
            double s = group.profile->get_position(current_time).to_inches();
            double kappa = group.query_curvature(s);
            double omega_ref = v_ref.to_ips() * kappa;

            // Build trajectory state for Ramsete
            trajectory::TrajectoryState ref_state;
            ref_state.x = target_pose(0);
            ref_state.y = target_pose(1);
            ref_state.theta = target_pose(2);
            ref_state.arc_velocity = v_ref;
            ref_state.omega = omega_ref;

            // Compute Ramsete control output
            control::RamseteOutput output = controller.compute(robot_pose, ref_state);

            // Compute time-varying gain for telemetry
            double omega_ref_sq = omega_ref * omega_ref;
            double v_ref_sq = v_ref.to_ips() * v_ref.to_ips();
            double k_gain = 2.0 * config_.controllers.ramsete.zeta *
                            std::sqrt(omega_ref_sq + config_.controllers.ramsete.b * v_ref_sq);

            // === UPDATE TELEMETRY ===
            update_ramsete_telemetry(ref_state, output, robot_pose, kappa, k_gain);
            update_pose_telemetry(robot_pose);

            // Convert body velocities to wheel velocities
            double half_track = track_width.to_inches() / 2.0;
            units::Velocity left_vel = units::Velocity::from_ips(
                output.v.to_ips() - output.omega.to_rad_per_sec() * half_track);
            units::Velocity right_vel = units::Velocity::from_ips(
                output.v.to_ips() + output.omega.to_rad_per_sec() * half_track);

            // Store wheel commands in telemetry
            {
                auto &telem = telemetry::g_telemetry.get_write_buffer();
                telem.left_wheel_cmd = left_vel;
                telem.right_wheel_cmd = right_vel;
            }

            // Command motors
            move_velocity(left_vel, right_vel);

            // === CRITICAL: SWAP BUFFERS ===
            telemetry::g_telemetry.swap();

            pros::delay(10);
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
        // Get current pose in STANDARD frame
        estimation::Pose current_standard = get_pose_standard();

        // Convert target from CORNER -> STANDARD
        estimation::Pose target_corner(
            target_x, target_y, target_heading,
            units::Velocity::from_ips(0),
            units::AngularVelocity::from_rad_per_sec(0));

        estimation::Pose target_standard = field::alliance_corner_to_standard(
            target_corner, alliance_, config_.field_config);

        // Create path poses in STANDARD frame
        path::Pose start(
            current_standard.x().to_inches(),
            current_standard.y().to_inches(),
            current_standard.theta().to_radians());

        path::Pose end(
            target_standard.x().to_inches(),
            target_standard.y().to_inches(),
            target_standard.theta().to_radians());

        // Build path with motion profile
        path::Path ramsete_path;
        path::ProfileGroup group(
            "ramsete_quintic",
            max_velocity,
            max_acceleration);

        auto segment = std::make_unique<path::QuinticHermiteSegment>(start, end);
        group.add_segment(std::move(segment));
        group.compute_arc_length();

        ramsete_path.add_profile_group(std::move(group));

        // Execute with Ramsete controller
        follow_path_ramsete(ramsete_path, timeout);
    }

} // namespace abclib::hardware