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
            auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
            telem = telemetry::TelemetryData{};
            telem.max_cross_track_error = units::Length::from_inches(0);
            telem.cumulative_xte = units::Length::from_inches(0);
            telem.max_along_track_error = units::Length::from_inches(0);
            telem.cumulative_ate = units::Length::from_inches(0);
            abclib::telemetry::g_telemetry.swap();
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
            auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
            telem = telemetry::TelemetryData{};
            telem.max_cross_track_error = units::Length::from_inches(0);
            telem.cumulative_xte = units::Length::from_inches(0);
            telem.max_along_track_error = units::Length::from_inches(0);
            telem.cumulative_ate = units::Length::from_inches(0);
            abclib::telemetry::g_telemetry.swap();
        }

        // Follow each profile group sequentially
        for (size_t i = 0; i < groups.size(); ++i)
        {
            const auto &group = groups[i];

            // Check if we've exceeded total timeout
            const uint32_t current_time = pros::millis();
            const units::Time elapsed = units::Time::from_milliseconds(current_time - start_time);

            if (elapsed >= timeout)
            {
                auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
                telem.settlement_reason = abclib::telemetry::SettlementReason::TIMEOUT;
                telem.time_to_settle = elapsed;
                telem.path_status = abclib::telemetry::PathFollowerStatus::COMPLETE;
                abclib::telemetry::g_telemetry.swap();
                return;
            }

            // Create trajectory for this profile group
            Trajectory trajectory(&group);

            // Calculate remaining time for this group
            units::Time remaining_time = units::Time::from_seconds(
                timeout.to_seconds() - elapsed.to_seconds());

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
            auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
            telem.path_status = abclib::telemetry::PathFollowerStatus::COMPLETE;
            abclib::telemetry::g_telemetry.swap();
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
            const units::Time elapsed_time = units::Time::from_milliseconds(current_time - start_time);

            // Check timeout
            if (elapsed_time >= config.timeout)
            {
                auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
                telem.settlement_reason = abclib::telemetry::SettlementReason::TIMEOUT;
                telem.time_to_settle = elapsed_time;
                telem.path_status = abclib::telemetry::PathFollowerStatus::COMPLETE;
                abclib::telemetry::g_telemetry.swap();
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
                    reference_state.theta - current_pose_body.theta_rad());
                double omega_feedback = config.turn_kP * heading_error;

                // Combined command
                units::AngularVelocity omega_command =
                    units::AngularVelocity::from_rad_per_sec(omega_ref + omega_feedback);

                // Convert to wheel velocities (v = 0 for turn-in-place)
                units::Length track_width = chassis_->get_track_width();
                wheel_vels = kinematics::diff_drive_ik(
                    units::Velocity::from_mps(0),
                    omega_command,
                    track_width);

                // Calculate wheel accelerations for feedforward
                double half_track = track_width.to_inches() / 2.0;
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
                ramsete_output.e_x = units::Length::from_inches(0);
                ramsete_output.e_y = units::Length::from_inches(0);
                ramsete_output.e_theta = units::Angle::from_radians(heading_error);
                ramsete_output.v = units::Velocity::from_mps(0);
                ramsete_output.omega = omega_command;

                // Update turn-specific telemetry
                {
                    auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
                    telem.omega_reference = units::AngularVelocity::from_rad_per_sec(omega_ref);
                    telem.omega_error = units::AngularVelocity::from_rad_per_sec(
                        omega_ref - current_pose_body.omega.to_rad_per_sec());
                    telem.omega_pid_output = omega_feedback;
                    telem.omega_commanded = omega_command;
                    telem.left_wheel_cmd = wheel_vels.left;
                    telem.right_wheel_cmd = wheel_vels.right;
                    abclib::telemetry::g_telemetry.swap();
                }

                // Approximate voltages for telemetry
                left_voltage = units::Voltage::from_volts(0);
                right_voltage = units::Voltage::from_volts(0);
            }
            else
            {
                // ===== NORMAL PATH CONTROL: RAMSETE =====

                // Convert current pose from body frame (REP-103) to math frame for RAMSETE
                units::Length current_x_math;
                units::Length current_y_math;
                units::Angle current_theta_math;

                // Create BodyPose from current pose
                math::BodyPose body_pose = math::BodyPose::from_inches_radians(
                    current_pose_body.x_inches(),
                    current_pose_body.y_inches(),
                    current_pose_body.theta_rad());

                // Convert to math frame
                math::body_to_math_frame(body_pose, current_x_math, current_y_math, current_theta_math);

                // Create math-frame pose for RAMSETE
                estimation::Pose current_pose_math(
                    current_x_math.to_inches(),
                    current_y_math.to_inches(),
                    current_theta_math.to_radians());
                current_pose_math.v = current_pose_body.v;
                current_pose_math.omega = current_pose_body.omega;

                // Compute RAMSETE control
                ramsete_output = ramsete_.compute(current_pose_math, reference_state);

                // Convert to wheel velocities
                units::Length track_width = chassis_->get_track_width();
                wheel_vels = kinematics::diff_drive_ik(
                    ramsete_output.v, ramsete_output.omega, track_width);

                // Calculate wheel accelerations - convert from SI to inches/s²
                double half_track = track_width.to_inches() / 2.0;
                double body_linear_accel = reference_state.arc_acceleration.to_mps2() / units::constants::INCH_TO_METER;
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
                    auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
                    telem.is_settled = true;
                    telem.settlement_reason = abclib::telemetry::SettlementReason::WITHIN_THRESHOLD;
                    telem.time_to_settle = elapsed_time;
                    telem.path_status = abclib::telemetry::PathFollowerStatus::COMPLETE;
                    abclib::telemetry::g_telemetry.swap();
                    break;
                }
            }

            // ===== STATUS AND TELEMETRY (common for both modes) =====

            telemetry::PathFollowerStatus current_status = determine_trajectory_status(
                trajectory, elapsed_time, settle_count > 0);

            update_telemetry(current_pose_body, reference_state, ramsete_output,
                             left_voltage, right_voltage,
                             current_status, elapsed_time, trajectory.get_total_time());

            pros::delay(10);
        }

        // Stop motors when complete
        chassis_->stop_motors();

        {
            auto &telem = abclib::telemetry::g_telemetry.get_write_buffer();
            telem.path_status = abclib::telemetry::PathFollowerStatus::IDLE;
            abclib::telemetry::g_telemetry.swap();
        }
    }

    telemetry::PathFollowerStatus PathFollower::determine_trajectory_status(
        const Trajectory &trajectory,
        units::Time elapsed_time,
        bool is_settling) const
    {
        if (is_settling)
        {
            return abclib::telemetry::PathFollowerStatus::SETTLING;
        }

        if (trajectory.is_complete(elapsed_time))
        {
            return abclib::telemetry::PathFollowerStatus::COMPLETE;
        }

        auto state = trajectory.get_state(elapsed_time);

        // Compare with typed units
        if (state.arc_acceleration > units::Acceleration::from_mps2(0.01))
        {
            return abclib::telemetry::PathFollowerStatus::ACCELERATING;
        }
        else if (state.arc_acceleration < units::Acceleration::from_mps2(-0.01))
        {
            return abclib::telemetry::PathFollowerStatus::DECELERATING;
        }
        else
        {
            return abclib::telemetry::PathFollowerStatus::CRUISING;
        }
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
                    time.to_seconds() - cumulative_time.to_seconds());

                // Return state from this group's trajectory
                TrajectoryState state = trajectory.get_state(relative_time);
                state.time = time; // Update to global time
                return state;
            }

            // Move to next group
            cumulative_time = units::Time::from_seconds(
                cumulative_time.to_seconds() + group_duration.to_seconds());
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