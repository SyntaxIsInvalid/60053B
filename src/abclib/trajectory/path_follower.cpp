#include "abclib/trajectory/path_follower.hpp"
#include "abclib/kinematics/differential_drive.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/math/angles.hpp"
#include <mutex>
#include "abclib/hardware/chassis.hpp"
#include <algorithm>
#include <cmath>
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

    void PathFollower::follow_segment(
        const path::IPathSegment *segment,
        const FollowerConfig &config)
    {
        if (!segment)
        {
            throw std::invalid_argument("PathFollower: segment pointer cannot be null");
        }

        Trajectory trajectory(segment, config.max_velocity, config.max_acceleration);

        {
            std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
            abclib::telemetry = TelemetryData{}; // Reset to default values

            // Initialize tracking error fields (already initialized to 0 by TelemetryData{})
            // But if you want to be explicit:
            abclib::telemetry.max_cross_track_error = units::Distance::from_inches(0);
            abclib::telemetry.cumulative_xte = units::Distance::from_inches(0);
            abclib::telemetry.max_along_track_error = units::Distance::from_inches(0);
            abclib::telemetry.cumulative_ate = units::Distance::from_inches(0);
        }

        execute_control_loop(segment, trajectory, config);
    }

    void PathFollower::execute_control_loop(
        const path::IPathSegment *segment,
        const Trajectory &trajectory,
        const FollowerConfig &config)
    {
        const uint32_t start_time = pros::millis();
        const double dt = 0.01;

        bool is_turn_in_place = segment->is_turn_in_place();

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

            estimation::Pose current_pose = chassis_->get_pose();

            // For turn-in-place: extend trajectory time slightly for final convergence
            units::Time sample_time = elapsed_time;
            if (is_turn_in_place && elapsed_time > trajectory.get_total_time())
            {
                sample_time = trajectory.get_total_time();
            }

            TrajectoryState reference_state = trajectory.get_state(sample_time);

            if (config.state_callback)
            {
                config.state_callback(reference_state);
            }

            // Exit condition: trajectory complete + reasonably close
            if (trajectory.is_complete(elapsed_time))
            {
                if (is_turn_in_place)
                {
                    double heading_error = math::normalize_angle(
                        reference_state.theta - current_pose.theta());

                    // Loose exit criteria: within 3 degrees
                    if (std::abs(heading_error) < math::deg_to_rad(1))
                    {
                        std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
                        abclib::telemetry.is_settled = true;
                        abclib::telemetry.settlement_reason = abclib::SettlementReason::WITHIN_THRESHOLD;
                        abclib::telemetry.time_to_settle = elapsed_time;
                        abclib::telemetry.path_status = abclib::PathFollowerStatus::COMPLETE;
                        break;
                    }
                }
                else
                {
                    // Normal path exit logic
                    double dx = reference_state.x - current_pose.x();
                    double dy = reference_state.y - current_pose.y();
                    units::Distance position_error = units::Distance::from_inches(
                        std::sqrt(dx * dx + dy * dy));

                    if (position_error.inches < config.position_threshold.inches)
                    {
                        std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
                        abclib::telemetry.is_settled = true;
                        abclib::telemetry.settlement_reason = abclib::SettlementReason::WITHIN_THRESHOLD;
                        abclib::telemetry.time_to_settle = elapsed_time;
                        abclib::telemetry.path_status = abclib::PathFollowerStatus::COMPLETE;
                        break;
                    }
                }
            }

            // Determine current status
            PathFollowerStatus current_status = determine_trajectory_status(
                trajectory, elapsed_time, false);

            // Single control law - no settling mode
            control::RamseteOutput ramsete_output;
            kinematics::WheelVelocities wheel_vels;
            units::Voltage left_voltage = units::Voltage::from_volts(0);
            units::Voltage right_voltage = units::Voltage::from_volts(0);

            if (is_turn_in_place)
            {
                double omega_ref = reference_state.omega;
                double omega_actual = current_pose.omega.rad_per_sec;

                // Heading correction
                double heading_error = math::normalize_angle(reference_state.theta - current_pose.theta());
                double heading_kp = 0.5;
                double omega_heading = heading_kp * heading_error;
                double omega_cmd = omega_ref + omega_heading;

                units::Distance track_width = chassis_->get_track_width();
                wheel_vels = kinematics::diff_drive_ik(
                    units::BodyLinearVelocity(0),
                    units::BodyAngularVelocity(omega_cmd),
                    track_width);

                const auto &chassis_config = chassis_->get_config();
                chassis_->move_velocity(wheel_vels.left, wheel_vels.right,
                                        chassis_config.turn_in_place_kS,
                                        chassis_config.turn_in_place_kV);

                // Build ramsete output for telemetry
                ramsete_output.v = units::BodyLinearVelocity(0);
                ramsete_output.omega = units::BodyAngularVelocity(omega_cmd);
                ramsete_output.e_x = units::Distance::from_inches(0);
                ramsete_output.e_y = units::Distance::from_inches(0);
                ramsete_output.e_theta = units::Radians(heading_error);

                // Turn-specific telemetry
                {
                    std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
                    abclib::telemetry.omega_reference = units::BodyAngularVelocity(omega_ref);
                    abclib::telemetry.omega_error = units::BodyAngularVelocity(omega_cmd - omega_actual);
                    abclib::telemetry.omega_pid_output = 0.0; // Not using PID anymore
                    abclib::telemetry.omega_commanded = units::BodyAngularVelocity(omega_cmd);
                    abclib::telemetry.left_wheel_cmd = wheel_vels.left;
                    abclib::telemetry.right_wheel_cmd = wheel_vels.right;
                }
            }
            else
            {
                // Normal RAMSETE path following
                ramsete_output = ramsete_.compute(current_pose, reference_state);

                units::Distance track_width = chassis_->get_track_width();
                wheel_vels = kinematics::diff_drive_ik(
                    ramsete_output.v, ramsete_output.omega, track_width);

                chassis_->move_velocity(wheel_vels.left, wheel_vels.right);
            }

            // Full telemetry update
            update_telemetry(current_pose, reference_state, ramsete_output,
                             left_voltage, right_voltage,
                             current_status, elapsed_time, trajectory.get_total_time());

            pros::delay(10);
        }

        chassis_->stop_motors();

        {
            std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
            abclib::telemetry.path_status = abclib::PathFollowerStatus::IDLE;
        }
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
            config.ramsete_constants = {2.0, 0.7}; // Use defaults or expose as parameter

            // Execute this profile group
            execute_control_loop(nullptr, trajectory, config);
        }

        // Mark path as complete
        {
            std::lock_guard<pros::Mutex> lock(abclib::telemetry_mutex);
            abclib::telemetry.path_status = abclib::PathFollowerStatus::COMPLETE;
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
        TrajectoryState final_state = last_trajectory.get_state(last_trajectory.get_total_time());
        final_state.time = time; // Keep requested time
        return final_state;
    }

    bool PathFollower::check_settlement(
        const estimation::Pose &current_pose,
        const TrajectoryState &reference_state,
        const FollowerConfig &config,
        int &settle_count) const
    {
        // For turn-in-place, only check angular error and angular velocity
        // For normal paths, check position error and linear velocity

        bool is_turn_in_place = (std::abs(reference_state.vx) < 1e-6 &&
                                 std::abs(reference_state.vy) < 1e-6);

        if (is_turn_in_place)
        {
            // Turn-in-place settlement criteria
            double angular_error = math::normalize_angle(
                reference_state.theta - current_pose.theta());

            bool heading_ok = std::abs(angular_error) < 0.017;                         // ~1 degree in radians
            bool angular_velocity_ok = std::abs(current_pose.omega.rad_per_sec) < 0.1; // rad/s

            if (heading_ok && angular_velocity_ok)
            {
                settle_count++;
                if (settle_count >= config.settle_count_required)
                {
                    return true;
                }
            }
            else
            {
                settle_count = 0;
            }
        }
        else
        {
            // Normal path settlement criteria
            double dx = reference_state.x - current_pose.x();
            double dy = reference_state.y - current_pose.y();
            units::Distance position_error = units::Distance::from_inches(std::sqrt(dx * dx + dy * dy));

            bool position_ok = position_error.inches < config.position_threshold.inches;
            bool velocity_ok = std::abs(current_pose.v.inches_per_sec) < config.velocity_threshold.inches_per_sec;

            if (position_ok && velocity_ok)
            {
                settle_count++;
                if (settle_count >= config.settle_count_required)
                {
                    return true;
                }
            }
            else
            {
                settle_count = 0;
            }
        }

        return false;
    }

    abclib::PathFollowerStatus PathFollower::determine_trajectory_status(
        const Trajectory &trajectory,
        units::Time elapsed_time, // TYPED
        bool is_settling) const
    {
        if (is_settling)
        {
            return abclib::PathFollowerStatus::SETTLING;
        }

        if (trajectory.is_complete(elapsed_time)) // Already accepts units::Time
        {
            return abclib::PathFollowerStatus::COMPLETE;
        }

        auto state = trajectory.get_state(elapsed_time); // Already accepts units::Time

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
        abclib::PathFollowerStatus status,
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
        abclib::telemetry.reference_arc_position = units::Distance::from_inches(reference_state.arc_length);

        // Current pose
        abclib::telemetry.pose = current_pose.pose;
        abclib::telemetry.pose_v = current_pose.v;
        abclib::telemetry.pose_omega = current_pose.omega;

        // Tracking errors - EXTRACT .inches from typed Distance
        abclib::telemetry.lateral_error = ramsete_output.e_y;     // Already typed Distance
        abclib::telemetry.angular_error = ramsete_output.e_theta; // Already typed Radians

        // Target and actual values - FIX: Extract .inches when doing arithmetic
        abclib::telemetry.lateral_target = units::Distance::from_inches(reference_state.arc_length);
        abclib::telemetry.lateral_actual = units::Distance::from_inches(
            reference_state.arc_length - ramsete_output.e_x.inches); // ← ADD .inches

        abclib::telemetry.angular_target = units::Radians(reference_state.theta);
        abclib::telemetry.angular_actual = units::Radians(current_pose.theta());

        // Motor voltages
        abclib::telemetry.left_motor_voltage = left_voltage;
        abclib::telemetry.right_motor_voltage = right_voltage;

        // Cross-track error (lateral error magnitude)
        // FIX: Extract .inches for arithmetic
        double abs_e_y = std::abs(ramsete_output.e_y.inches); // ← ADD .inches

        abclib::telemetry.cross_track_error = units::Distance::from_inches(abs_e_y);
        abclib::telemetry.max_cross_track_error = units::Distance::from_inches(
            std::max(abclib::telemetry.max_cross_track_error.inches, abs_e_y));
        abclib::telemetry.cumulative_xte = units::Distance::from_inches(
            abclib::telemetry.cumulative_xte.inches + abs_e_y * 0.01);

        // Along-track error (longitudinal error magnitude)
        // FIX: Extract .inches for arithmetic
        double abs_e_x = std::abs(ramsete_output.e_x.inches); // ← ADD .inches

        abclib::telemetry.along_track_error = units::Distance::from_inches(abs_e_x);
        abclib::telemetry.max_along_track_error = units::Distance::from_inches(
            std::max(abclib::telemetry.max_along_track_error.inches, abs_e_x));
        abclib::telemetry.cumulative_ate = units::Distance::from_inches(
            abclib::telemetry.cumulative_ate.inches + abs_e_x * 0.01);

        // PID terms (not used by RAMSETE, set to zero)
        abclib::telemetry.lateral_p_term = 0;
        abclib::telemetry.lateral_i_term = 0;
        abclib::telemetry.lateral_d_term = 0;
        abclib::telemetry.angular_p_term = 0;
        abclib::telemetry.angular_i_term = 0;
        abclib::telemetry.angular_d_term = 0;
        abclib::telemetry.lateral_output = units::Voltage::from_volts(0);
        abclib::telemetry.angular_output = units::Voltage::from_volts(0);
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