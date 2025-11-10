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
                reference_state.theta - current_pose.theta());

            bool heading_ok = std::abs(angular_error) < 0.017; // ~1 degree
            bool angular_velocity_ok = std::abs(current_pose.omega.rad_per_sec) < 0.1;

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
            units::Distance position_error = units::Distance::from_inches(
                std::sqrt(ramsete_output.e_x.inches * ramsete_output.e_x.inches +
                          ramsete_output.e_y.inches * ramsete_output.e_y.inches));

            bool position_ok = position_error.inches < config.position_threshold.inches;
            bool velocity_ok = std::abs(current_pose.v.inches_per_sec) <
                               config.velocity_threshold.inches_per_sec;

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
        PathFollowerStatus status,
        units::Time elapsed_time,
        units::Time total_time) const
    {
        // Get write buffer reference
        auto& telem = abclib::telemetry.get_write_buffer();

        // Path status and timing
        telem.path_status = status;
        telem.trajectory_time = elapsed_time;
        telem.trajectory_progress = std::clamp(
            elapsed_time.seconds / total_time.seconds, 0.0, 1.0);
        telem.trajectory_total_time = total_time;

        // Reference values
        telem.reference_velocity = reference_state.arc_velocity;
        telem.reference_arc_position = units::Distance::from_inches(
            reference_state.arc_length);

        // Current pose
        telem.pose = current_pose.pose;
        telem.pose_v = current_pose.v;
        telem.pose_omega = current_pose.omega;

        // Tracking errors
        telem.lateral_error = ramsete_output.e_y;
        telem.angular_error = ramsete_output.e_theta;

        // Target and actual values
        telem.lateral_target = units::Distance::from_inches(
            reference_state.arc_length);
        telem.lateral_actual = units::Distance::from_inches(
            reference_state.arc_length - ramsete_output.e_x.inches);

        telem.angular_target = units::Radians(reference_state.theta);
        telem.angular_actual = units::Radians(current_pose.theta());

        // Motor voltages
        telem.left_motor_voltage = left_voltage;
        telem.right_motor_voltage = right_voltage;

        // Cross-track error
        double abs_e_y = std::abs(ramsete_output.e_y.inches);
        telem.cross_track_error = units::Distance::from_inches(abs_e_y);
        telem.max_cross_track_error = units::Distance::from_inches(
            std::max(telem.max_cross_track_error.inches, abs_e_y));
        telem.cumulative_xte = units::Distance::from_inches(
            telem.cumulative_xte.inches + abs_e_y * 0.01);

        // Along-track error
        double abs_e_x = std::abs(ramsete_output.e_x.inches);
        telem.along_track_error = units::Distance::from_inches(abs_e_x);
        telem.max_along_track_error = units::Distance::from_inches(
            std::max(telem.max_along_track_error.inches, abs_e_x));
        telem.cumulative_ate = units::Distance::from_inches(
            telem.cumulative_ate.inches + abs_e_x * 0.01);

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
        abclib::telemetry.swap();
    }
}