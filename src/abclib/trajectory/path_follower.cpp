#include "abclib/trajectory/path_follower.hpp"
#include "abclib/kinematics/differential_drive.hpp"
#include "abclib/hardware/chassis.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/math/angles.hpp"
#include <mutex>
#include <algorithm>
#include <cmath>
#include "abclib/math/coordinate_frames.hpp"

namespace abclib::trajectory
{
    PathFollower::PathFollower(hardware::Chassis *chassis,
                               const control::RamseteConstants &constants)
        : chassis_(chassis),
          ramsete_(constants)
    {
        if (!chassis_)
        {
            throw std::invalid_argument("PathFollower: chassis pointer cannot be null");
        }
    }

    void PathFollower::follow_segment(const path::IPathSegment *segment,
                                      const FollowerConfig &config)
    {
        if (!segment)
        {
            throw std::invalid_argument("PathFollower: segment pointer cannot be null");
        }

        // Create trajectory from single segment
        Trajectory trajectory(segment, config.max_velocity, config.max_acceleration);

        // Reset telemetry
        {
            std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
            abclib::telemetry = TelemetryData{};
            abclib::telemetry.max_cross_track_error = units::Distance::from_inches(0);
            abclib::telemetry.cumulative_xte = units::Distance::from_inches(0);
            abclib::telemetry.max_along_track_error = units::Distance::from_inches(0);
            abclib::telemetry.cumulative_ate = units::Distance::from_inches(0);
        }

        ramsete_.set_constants(config.ramsete_constants);

        // Execute
        execute_trajectory(trajectory, config);
    }

    void PathFollower::follow_path(const path::Path &path, units::Time timeout)
    {
        if (path.empty())
        {
            throw std::invalid_argument("PathFollower: cannot follow empty path");
        }

        const auto &groups = path.get_profile_groups();
        const uint32_t start_time = pros::millis();

        // Reset telemetry for entire path
        {
            std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
            abclib::telemetry = TelemetryData{};
            abclib::telemetry.max_cross_track_error = units::Distance::from_inches(0);
            abclib::telemetry.cumulative_xte = units::Distance::from_inches(0);
            abclib::telemetry.max_along_track_error = units::Distance::from_inches(0);
            abclib::telemetry.cumulative_ate = units::Distance::from_inches(0);
        }

        // Follow each profile group sequentially
        for (size_t i = 0; i < groups.size(); ++i)
        {
            const auto &group = groups[i];

            // Check if we've exceeded total timeout
            const uint32_t current_time = pros::millis();
            const units::Time elapsed = units::Time::from_millis(current_time - start_time);

            if (elapsed >= timeout)
            {
                std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
                abclib::telemetry.settlement_reason = abclib::SettlementReason::TIMEOUT;
                abclib::telemetry.time_to_settle = elapsed;
                abclib::telemetry.path_status = abclib::PathFollowerStatus::COMPLETE;
                return;
            }

            // Create trajectory for this profile group
            Trajectory trajectory(&group);

            // Calculate remaining time for this group
            units::Time remaining_time = units::Time::from_seconds(
                timeout.seconds - elapsed.seconds);

            // Create config for this group
            FollowerConfig config;
            config.max_velocity = group.max_velocity;
            config.max_acceleration = group.max_acceleration;
            config.timeout = remaining_time;
            config.ramsete_constants = ramsete_.get_constants();

            // Execute this profile group
            execute_trajectory(trajectory, config);
        }

        // Mark path as complete
        {
            std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
            abclib::telemetry.path_status = abclib::PathFollowerStatus::COMPLETE;
        }
    }

    void PathFollower::execute_trajectory(const Trajectory &trajectory,
                                          const FollowerConfig &config)
    {
        const uint32_t start_time = pros::millis();
        int settle_count = 0;

        // Set RAMSETE constants from config
        ramsete_.set_constants(config.ramsete_constants);

        while (true)
        {
            const uint32_t current_time = pros::millis();
            const units::Time elapsed_time = units::Time::from_millis(current_time - start_time);

            // Check timeout
            if (elapsed_time >= config.timeout)
            {
                std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
                abclib::telemetry.settlement_reason = abclib::SettlementReason::TIMEOUT;
                abclib::telemetry.time_to_settle = elapsed_time;
                abclib::telemetry.path_status = abclib::PathFollowerStatus::COMPLETE;
                break;
            }

            // Get current state
            estimation::Pose current_pose_body = chassis_->get_pose();

            // Get reference state from trajectory (math frame)
            units::Time sample_time = std::min(elapsed_time, trajectory.get_total_time());
            TrajectoryState reference_state = trajectory.get_state(sample_time);

            // Get current segment to determine control mode
            const path::IPathSegment *current_seg = trajectory.get_current_segment(reference_state.arc_length);

            // Call state callback if provided
            if (config.state_callback)
            {
                config.state_callback(reference_state);
            }

            // Branch on segment type for control computation
            kinematics::WheelVelocities wheel_vels;
            units::Voltage left_voltage;
            units::Voltage right_voltage;
            control::RamseteOutput ramsete_output; // For telemetry and settlement

            if (current_seg->is_turn_in_place())
            {
                // ===== TURN-IN-PLACE CONTROL: Feedforward + Feedback =====

                // Feedforward: use reference omega from trajectory
                double omega_ref = reference_state.omega;

                // Feedback: proportional control on heading error
                double heading_error = math::normalize_angle(
                    reference_state.theta - current_pose_body.theta());
                double omega_feedback = config.turn_kP * heading_error;

                // Combined command
                units::BodyAngularVelocity omega_command =
                    units::BodyAngularVelocity(omega_ref + omega_feedback);

                // Convert to wheel velocities (v = 0 for turn-in-place)
                units::Distance track_width = chassis_->get_track_width();
                wheel_vels = kinematics::diff_drive_ik(
                    units::BodyLinearVelocity(0),
                    omega_command,
                    track_width);

                // Calculate wheel accelerations for feedforward
                double half_track = track_width.inches / 2.0;
                double body_angular_accel = reference_state.alpha;
                double left_accel = -half_track * body_angular_accel;
                double right_accel = half_track * body_angular_accel;

                // Apply control with turn-in-place gains
                const auto &chassis_config = chassis_->get_config();
                if (chassis_config.use_pros_controller)
                {
                    chassis_->move_velocity_pros(wheel_vels.left, wheel_vels.right);
                }
                else
                {
                    chassis_->move_velocity(wheel_vels.left, wheel_vels.right,
                                            left_accel, right_accel,
                                            chassis_config.turn_in_place_kS,
                                            chassis_config.turn_in_place_kV,
                                            chassis_config.turn_in_place_kA);
                }

                // Populate ramsete_output for settlement checking and telemetry
                // For turn-in-place, position errors are not meaningful
                ramsete_output.e_x = units::Distance::from_inches(0);
                ramsete_output.e_y = units::Distance::from_inches(0);
                ramsete_output.e_theta = units::Radians(heading_error);
                ramsete_output.v = units::BodyLinearVelocity(0);
                ramsete_output.omega = omega_command;

                // Update turn-specific telemetry
                {
                    std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
                    abclib::telemetry.omega_reference = units::BodyAngularVelocity(omega_ref);
                    abclib::telemetry.omega_error = units::BodyAngularVelocity(omega_ref - current_pose_body.omega.rad_per_sec);
                    abclib::telemetry.omega_pid_output = omega_feedback;
                    abclib::telemetry.omega_commanded = omega_command;
                    abclib::telemetry.left_wheel_cmd = wheel_vels.left;
                    abclib::telemetry.right_wheel_cmd = wheel_vels.right;
                }

                // Approximate voltages for telemetry
                left_voltage = units::Voltage::from_volts(0);
                right_voltage = units::Voltage::from_volts(0);
            }
            else
            {
                // ===== NORMAL PATH CONTROL: RAMSETE =====

                // Convert current pose from body frame (REP-103) to math frame for RAMSETE
                double current_x_math, current_y_math, current_theta_math;
                math::body_to_math_frame(current_pose_body.pose,
                                         current_x_math, current_y_math, current_theta_math);

                // Create math-frame pose for RAMSETE
                estimation::Pose current_pose_math;
                current_pose_math.pose = units::BodyPose::from_radians(current_x_math, current_y_math, current_theta_math);
                current_pose_math.v = current_pose_body.v;
                current_pose_math.omega = current_pose_body.omega;

                // Compute RAMSETE control
                ramsete_output = ramsete_.compute(current_pose_math, reference_state);

                // Convert to wheel velocities
                units::Distance track_width = chassis_->get_track_width();
                wheel_vels = kinematics::diff_drive_ik(
                    ramsete_output.v, ramsete_output.omega, track_width);

                // Calculate wheel accelerations
                double half_track = track_width.inches / 2.0;
                double body_linear_accel = reference_state.arc_acceleration;
                double body_angular_accel = reference_state.alpha;
                double left_accel = body_linear_accel - half_track * body_angular_accel;
                double right_accel = body_linear_accel + half_track * body_angular_accel;

                // Apply control
                const auto &chassis_config = chassis_->get_config();
                if (chassis_config.use_pros_controller)
                {
                    chassis_->move_velocity_pros(wheel_vels.left, wheel_vels.right);
                }
                else
                {
                    chassis_->move_velocity(wheel_vels.left, wheel_vels.right,
                                            left_accel, right_accel);
                }

                // Approximate voltages for telemetry
                left_voltage = units::Voltage::from_volts(0);
                right_voltage = units::Voltage::from_volts(0);
            }

            // ===== SETTLEMENT CHECKING (common for both modes) =====

            if (trajectory.is_complete(elapsed_time))
            {
                if (check_settlement(current_pose_body, reference_state, config,
                                     ramsete_output, settle_count, current_seg))
                {
                    std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
                    abclib::telemetry.is_settled = true;
                    abclib::telemetry.settlement_reason = abclib::SettlementReason::WITHIN_THRESHOLD;
                    abclib::telemetry.time_to_settle = elapsed_time;
                    abclib::telemetry.path_status = abclib::PathFollowerStatus::COMPLETE;
                    break;
                }
            }

            // ===== STATUS AND TELEMETRY (common for both modes) =====

            PathFollowerStatus current_status = determine_trajectory_status(
                trajectory, elapsed_time, settle_count > 0);

            update_telemetry(current_pose_body, reference_state, ramsete_output,
                             left_voltage, right_voltage,
                             current_status, elapsed_time, trajectory.get_total_time());

            pros::delay(10);
        }

        // Stop motors when complete
        chassis_->stop_motors();

        {
            std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
            abclib::telemetry.path_status = abclib::PathFollowerStatus::IDLE;
        }
    }

    bool PathFollower::check_settlement(
        const estimation::Pose &current_pose,
        const TrajectoryState &reference_state,
        const FollowerConfig &config,
        const control::RamseteOutput &ramsete_output, // ADD THIS PARAMETER
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

    PathFollowerStatus PathFollower::determine_trajectory_status(
        const Trajectory &trajectory,
        units::Time elapsed_time,
        bool is_settling) const
    {
        if (is_settling)
        {
            return abclib::PathFollowerStatus::SETTLING;
        }

        if (trajectory.is_complete(elapsed_time))
        {
            return abclib::PathFollowerStatus::COMPLETE;
        }

        auto state = trajectory.get_state(elapsed_time);

        if (state.arc_acceleration > 0.01)
        {
            return abclib::PathFollowerStatus::ACCELERATING;
        }
        else if (state.arc_acceleration < -0.01)
        {
            return abclib::PathFollowerStatus::DECELERATING;
        }
        else
        {
            return abclib::PathFollowerStatus::CRUISING;
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
        std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);

        // Path status and timing
        abclib::telemetry.path_status = status;
        abclib::telemetry.trajectory_time = elapsed_time;
        abclib::telemetry.trajectory_progress = std::clamp(
            elapsed_time.seconds / total_time.seconds, 0.0, 1.0);
        abclib::telemetry.trajectory_total_time = total_time;

        // Reference values
        abclib::telemetry.reference_velocity = reference_state.arc_velocity;
        abclib::telemetry.reference_arc_position = units::Distance::from_inches(
            reference_state.arc_length);

        // Current pose
        abclib::telemetry.pose = current_pose.pose;
        abclib::telemetry.pose_v = current_pose.v;
        abclib::telemetry.pose_omega = current_pose.omega;

        // Tracking errors
        abclib::telemetry.lateral_error = ramsete_output.e_y;
        abclib::telemetry.angular_error = ramsete_output.e_theta;

        // Target and actual values
        abclib::telemetry.lateral_target = units::Distance::from_inches(
            reference_state.arc_length);
        abclib::telemetry.lateral_actual = units::Distance::from_inches(
            reference_state.arc_length - ramsete_output.e_x.inches);

        abclib::telemetry.angular_target = units::Radians(reference_state.theta);
        abclib::telemetry.angular_actual = units::Radians(current_pose.theta());

        // Motor voltages
        abclib::telemetry.left_motor_voltage = left_voltage;
        abclib::telemetry.right_motor_voltage = right_voltage;

        // Cross-track error
        double abs_e_y = std::abs(ramsete_output.e_y.inches);
        abclib::telemetry.cross_track_error = units::Distance::from_inches(abs_e_y);
        abclib::telemetry.max_cross_track_error = units::Distance::from_inches(
            std::max(abclib::telemetry.max_cross_track_error.inches, abs_e_y));
        abclib::telemetry.cumulative_xte = units::Distance::from_inches(
            abclib::telemetry.cumulative_xte.inches + abs_e_y * 0.01);

        // Along-track error
        double abs_e_x = std::abs(ramsete_output.e_x.inches);
        abclib::telemetry.along_track_error = units::Distance::from_inches(abs_e_x);
        abclib::telemetry.max_along_track_error = units::Distance::from_inches(
            std::max(abclib::telemetry.max_along_track_error.inches, abs_e_x));
        abclib::telemetry.cumulative_ate = units::Distance::from_inches(
            abclib::telemetry.cumulative_ate.inches + abs_e_x * 0.01);

        // Zero out PID terms (RAMSETE doesn't use them)
        abclib::telemetry.lateral_p_term = 0;
        abclib::telemetry.lateral_i_term = 0;
        abclib::telemetry.lateral_d_term = 0;
        abclib::telemetry.angular_p_term = 0;
        abclib::telemetry.angular_i_term = 0;
        abclib::telemetry.angular_d_term = 0;
        abclib::telemetry.lateral_output = units::Voltage::from_volts(0);
        abclib::telemetry.angular_output = units::Voltage::from_volts(0);
    }

    TrajectoryState PathFollower::get_state_at(const path::Path &path, units::Time time) const
    {
        if (path.empty())
        {
            throw std::invalid_argument("PathFollower: cannot query empty path");
        }

        const auto &groups = path.get_profile_groups();

        // Track cumulative time across profile groups
        units::Time cumulative_time = units::Time::from_seconds(0);

        for (const auto &group : groups)
        {
            // Create trajectory for this group to get timing info
            Trajectory trajectory(&group);
            units::Time group_duration = trajectory.get_total_time();

            // Check if requested time falls within this group
            if (time < cumulative_time + group_duration)
            {
                // Time is within this group - calculate relative time
                units::Time relative_time = units::Time::from_seconds(
                    time.seconds - cumulative_time.seconds);

                // Return state from this group's trajectory
                TrajectoryState state = trajectory.get_state(relative_time);
                state.time = time; // Update to global time
                return state;
            }

            // Move to next group
            cumulative_time = units::Time::from_seconds(
                cumulative_time.seconds + group_duration.seconds);
        }

        // Time is beyond path end - return final state
        const auto &last_group = groups.back();
        Trajectory last_trajectory(&last_group);
        TrajectoryState final_state = last_trajectory.get_state(
            last_trajectory.get_total_time());
        final_state.time = time;
        return final_state;
    }

    void PathFollower::set_ramsete_constants(const control::RamseteConstants &constants)
    {
        ramsete_.set_constants(constants);
    }

    control::RamseteConstants PathFollower::get_ramsete_constants() const
    {
        return ramsete_.get_constants();
    }

} // namespace abclib::trajectory