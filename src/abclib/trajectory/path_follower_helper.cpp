#include "abclib/trajectory/path_follower.hpp"

namespace abclib::trajectory
{
    bool PathFollower::check_settlement(
        const estimation::Pose &current_pose,
        const TrajectoryState &reference_state,
        const FollowerConfig &config,
        const control::RamseteOutput &ramsete_output,
        int &settle_count,
        const path::IPathSegment *segment) const
    {
        // Detect turn-in-place from reference state velocities
        if (segment->is_turn_in_place())
        {
            // Turn-in-place settlement criteria
            double angular_error = math::normalize_angle(
                reference_state.theta - current_pose.theta_rad());

            bool heading_ok = std::abs(angular_error) < 0.017; // ~1 degree
            bool angular_velocity_ok = std::abs(current_pose.omega.to_rad_per_sec()) < 0.1;

            if (heading_ok && angular_velocity_ok)
            {
                settle_count++;
                return settle_count >= config.settle_count_required;
            }
            else
            {
                settle_count = 0;
                return false;
            }
        }
        else
        {
            // Normal path settlement criteria
            units::Length position_error = units::Length::from_inches(
                std::sqrt(ramsete_output.e_x.to_inches() * ramsete_output.e_x.to_inches() +
                          ramsete_output.e_y.to_inches() * ramsete_output.e_y.to_inches()));

            bool position_ok = position_error.to_inches() < config.position_threshold.to_inches();
            bool velocity_ok = std::abs(current_pose.v.to_ips()) < config.velocity_threshold.to_ips();

            if (position_ok && velocity_ok)
            {
                settle_count++;
                return settle_count >= config.settle_count_required;
            }
            else
            {
                settle_count = 0;
                return false;
            }
        }
    }

    void PathFollower::update_telemetry(
        const estimation::Pose &current_pose,
        const TrajectoryState &reference_state,
        const control::RamseteOutput &ramsete_output,
        units::Voltage left_voltage,
        units::Voltage right_voltage,
        telemetry::PathFollowerStatus status,
        units::Time elapsed_time,
        units::Time total_time) const
    {
        // Get write buffer reference
        auto& telem = abclib::telemetry::g_telemetry.get_write_buffer();

        // Path status and timing
        telem.path_status = status;
        telem.trajectory_time = elapsed_time;
        telem.trajectory_progress = std::clamp(
            elapsed_time.to_seconds() / total_time.to_seconds(), 0.0, 1.0);
        telem.trajectory_total_time = total_time;

        // Reference values
        telem.reference_velocity = reference_state.arc_velocity;
        telem.reference_arc_position = units::Length::from_inches(reference_state.arc_length);

        // Current pose
        telem.pose = current_pose;

        // Tracking errors
        telem.lateral_error = ramsete_output.e_y;
        telem.angular_error = ramsete_output.e_theta;

        // Target and actual values
        telem.lateral_target = units::Length::from_inches(reference_state.arc_length);
        telem.lateral_actual = units::Length::from_inches(
            reference_state.arc_length - ramsete_output.e_x.to_inches());

        telem.angular_target = units::Angle::from_radians(reference_state.theta);
        telem.angular_actual = units::Angle::from_radians(current_pose.theta_rad());

        // Motor voltages
        telem.left_motor_voltage = left_voltage;
        telem.right_motor_voltage = right_voltage;

        // Cross-track error
        double abs_e_y = std::abs(ramsete_output.e_y.to_inches());
        telem.cross_track_error = units::Length::from_inches(abs_e_y);
        telem.max_cross_track_error = units::Length::from_inches(
            std::max(telem.max_cross_track_error.to_inches(), abs_e_y));
        telem.cumulative_xte = units::Length::from_inches(
            telem.cumulative_xte.to_inches() + abs_e_y * 0.01);

        // Along-track error
        double abs_e_x = std::abs(ramsete_output.e_x.to_inches());
        telem.along_track_error = units::Length::from_inches(abs_e_x);
        telem.max_along_track_error = units::Length::from_inches(
            std::max(telem.max_along_track_error.to_inches(), abs_e_x));
        telem.cumulative_ate = units::Length::from_inches(
            telem.cumulative_ate.to_inches() + abs_e_x * 0.01);

        // Zero out PID terms (RAMSETE doesn't use them)
        telem.lateral_p_term = 0;
        telem.lateral_i_term = 0;
        telem.lateral_d_term = 0;
        telem.angular_p_term = 0;
        telem.angular_i_term = 0;
        telem.angular_d_term = 0;
        telem.lateral_output = units::Voltage::from_volts(0);
        telem.angular_output = units::Voltage::from_volts(0);

        // Swap buffers to make updates visible to readers
        abclib::telemetry::g_telemetry.swap();
    }
}