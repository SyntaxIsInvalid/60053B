#include "abclib/hardware/chassis.hpp"
#include "api.h"
#include <mutex>
#include "abclib/path/straight_segment.hpp"
#include "abclib/path/turn_in_place_segment.hpp"
#include <fstream>
#include "abclib/kinematics/differential_drive.hpp"
#include "abclib/estimation/wheel_measurement_models.hpp"
#include "abclib/estimation/imu_measurement_model.hpp"
#include "abclib/math/coordinate_frames.hpp"
#include "abclib/path/straight_segment.hpp"
#include "abclib/telemetry/logger.hpp"
#include "abclib/trajectory/trajectory.hpp"
#include "abclib/control/profiled_pid.hpp"
#include "abclib/math/point.hpp"
using namespace abclib;

namespace abclib::hardware
{
    void Chassis::move_straight_profiled(
        units::Distance distance,
        units::BodyLinearVelocity max_velocity,
        double max_acceleration,
        units::Time timeout,
        double heading_tolerance)
    {
        // Get current pose from odometry (REP-103 body frame)
        estimation::Pose current_body = get_pose();

        // Convert current pose to math frame
        double start_x_math, start_y_math, start_theta_math;
        math::body_to_math_frame(current_body.pose,
                                 start_x_math, start_y_math, start_theta_math);

        // Calculate end pose in math frame
        // Move in the direction of current heading
        double end_x_math = start_x_math + distance.inches * std::cos(start_theta_math);
        double end_y_math = start_y_math + distance.inches * std::sin(start_theta_math);
        double end_theta_math = start_theta_math; // Heading stays the same

        // Create path segment in math frame
        path::Pose start_pose(start_x_math, start_y_math, start_theta_math);
        path::Pose end_pose(end_x_math, end_y_math, end_theta_math);

        path::StraightSegment segment(start_pose, end_pose, heading_tolerance);

        // Configure trajectory follower
        trajectory::FollowerConfig config;
        config.max_velocity = max_velocity;
        config.max_acceleration = max_acceleration;
        config.timeout = timeout;
        config.ramsete_constants = config_.ramsete_constants;

        // Execute the trajectory
        path_follower_->follow_segment(&segment, config);
    }

    void Chassis::turn_to_heading_profiled(
        units::Degrees target_heading,
        double max_body_angular_velocity_deg_per_sec,
        double max_body_angular_acceleration_deg_per_sec2,
        units::Time timeout)
    {
        // 1. Get current pose from odometry
        estimation::Pose current_pose = get_pose();
        double current_heading_rad = current_pose.theta();
        double target_heading_rad = target_heading.to_radians().value;

        // 2. Create TurnInPlaceSegment in math frame
        path::Pose start_pose(current_pose.x(), current_pose.y(), current_heading_rad);
        path::TurnInPlaceSegment turn_segment(start_pose, target_heading_rad, track_width);

        // 3. Convert angular velocity/acceleration to linear (wheel velocity)
        // For turn-in-place: v_wheel = ω_body * r, where r = track_width / 2
        double turning_radius = track_width.inches / 2.0;
        double max_angular_vel_rad_per_sec = max_body_angular_velocity_deg_per_sec * M_PI / 180.0;
        double max_angular_accel_rad_per_sec2 = max_body_angular_acceleration_deg_per_sec2 * M_PI / 180.0;

        double max_wheel_linear_velocity = max_angular_vel_rad_per_sec * turning_radius;
        double max_wheel_linear_accel = max_angular_accel_rad_per_sec2 * turning_radius;

        // 4. Configure follower with converted linear velocities
        trajectory::FollowerConfig config;
        config.max_velocity = units::BodyLinearVelocity(max_wheel_linear_velocity);
        config.max_acceleration = max_wheel_linear_accel;
        config.timeout = timeout;
        config.ramsete_constants = config_.ramsete_constants;
        config.turn_kP = 0.35; // Can make this configurable via ChassisConfig later

        // 5. Let the path follower handle everything!
        path_follower_->follow_segment(&turn_segment, config);
    }

    void Chassis::turn_to_heading_profiled_pid(
        units::Degrees target_heading,
        double max_angular_velocity_deg_per_sec,
        double max_angular_acceleration_deg_per_sec2,
        units::Time timeout)
    {
        double max_angular_velocity_rad_per_sec = max_angular_velocity_deg_per_sec * M_PI / 180.0;
        double max_angular_acceleration_rad_per_sec2 = max_angular_acceleration_deg_per_sec2 * M_PI / 180.0;

        std::uint32_t start_time = pros::millis();
        const double dt = 0.01;

        // Convert target heading to radians
        units::Radians target_heading_rad = target_heading.to_radians();
        double target_rad = target_heading_rad.value;

        // Get initial heading
        units::BodyHeading initial_heading = get_heading();
        double initial_rad = initial_heading.angle.value;
        double last_wrapped_rad = initial_rad;
        double cumulative_unwrapped_rad = initial_rad;

        // Unwrap target to be close to initial
        double angle_diff = target_rad - initial_rad;
        angle_diff = math::normalize_angle(angle_diff);
        double unwrapped_target = initial_rad + angle_diff;

        // Create ProfiledPID
        control::ProfiledPIDConstants profiled_constants;
        profiled_constants.pid_constants = config_.profiled_turn_pid_constants;
        profiled_constants.max_velocity = max_angular_velocity_rad_per_sec;
        profiled_constants.max_acceleration = max_angular_acceleration_rad_per_sec2;
        profiled_constants.position_tolerance = settlement_config_.angular_threshold.value;
        profiled_constants.velocity_tolerance = settlement_config_.angular_velocity_threshold.rad_per_sec;

        control::ProfiledPID profiled_pid(profiled_constants);
        profiled_pid.reset(cumulative_unwrapped_rad);

        // Reset telemetry
        reset_telemetry_accumulators();

        // Settlement tracking
        int settle_count = 0;
        const int REQUIRED_SETTLE_COUNT = 3;

        while ((pros::millis() - start_time) < timeout.to_millis_uint())
        {
            // Get current wrapped heading
            units::BodyHeading current_heading = get_heading();
            double current_wrapped_rad = current_heading.angle.value;

            // Calculate the delta and unwrap it
            double delta_rad = current_wrapped_rad - last_wrapped_rad;
            delta_rad = math::normalize_angle(delta_rad);
            cumulative_unwrapped_rad += delta_rad;
            last_wrapped_rad = current_wrapped_rad;

            estimation::Pose current_pose = get_pose();

            // Compute profiled PID output
            double angular_output = profiled_pid.compute(cumulative_unwrapped_rad, unwrapped_target, dt);

            // Calculate feedforward with deadband
            double target_velocity = profiled_pid.get_setpoint_velocity();
            double target_acceleration = profiled_pid.get_setpoint().acceleration;
            double ff_sign = (std::abs(target_velocity) < 0.01) ? 0.0 : math::sgn(target_velocity);
            double ff = config_.turn_in_place_kS * ff_sign +
                        config_.turn_in_place_kV * target_velocity +
                        config_.turn_in_place_kA * target_acceleration;

            angular_output += ff;

            // Clamp output
            angular_output = std::clamp(angular_output, -12.0, 12.0);

            // Calculate wrapped error for telemetry
            double angular_error_rad = unwrapped_target - cumulative_unwrapped_rad;
            angular_error_rad = math::normalize_angle(angular_error_rad);

            // Check settlement with counter
            if (profiled_pid.at_goal())
            {
                settle_count++;
                if (settle_count >= REQUIRED_SETTLE_COUNT)
                {
                    update_settlement_telemetry(true, settle_count,
                                                SettlementReason::WITHIN_THRESHOLD, start_time);
                    left_motors->brake();
                    right_motors->brake();
                    break;
                }
            }
            else
            {
                settle_count = 0;
            }

            // Motor voltages
            units::Voltage left_voltage = units::Voltage(-angular_output);
            units::Voltage right_voltage = units::Voltage(angular_output);

            // Update telemetry
            update_angular_telemetry(angular_error_rad, angular_output,
                                     unwrapped_target, cumulative_unwrapped_rad, dt);
            update_pose_telemetry(current_pose);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, settle_count, SettlementReason::NOT_SETTLED, start_time);

            move_left_motors(left_voltage);
            move_right_motors(right_voltage);
            telemetry.swap();
            pros::delay(10);
        }

        // Timeout check
        bool timed_out = (pros::millis() - start_time) >= timeout.to_millis_uint();
        if (timed_out)
        {
            const auto &current_data = telemetry.get_read_buffer();
            if (!current_data.is_settled)
            {
                update_settlement_telemetry(false, settle_count, SettlementReason::TIMEOUT, start_time);
            }
        }

        left_motors->brake();
        right_motors->brake();
    }

    void Chassis::drive_straight_profiled_pid(
        units::Distance target_distance,
        double max_velocity_inches_per_sec,
        double max_acceleration_inches_per_sec2,
        units::Time timeout,
        bool reset_position)
    {
        std::uint32_t start_time = pros::millis();
        const double dt = 0.01; // 100Hz

        if (reset_position)
        {
            reset_chassis_position();
        }

        estimation::Pose start_pose = get_pose();
        double start_heading = start_pose.theta();
        double initial_x = start_pose.x();
        double initial_y = start_pose.y();
        int settle_count = 0;

        // Create ProfiledPID for lateral control
        control::ProfiledPIDConstants profiled_constants;
        profiled_constants.pid_constants = config_.profiled_lateral_pid_constants;
        profiled_constants.max_velocity = max_velocity_inches_per_sec;
        profiled_constants.max_acceleration = max_acceleration_inches_per_sec2;
        profiled_constants.position_tolerance = settlement_config_.position_threshold.inches;
        profiled_constants.velocity_tolerance = settlement_config_.linear_velocity_threshold.inches_per_sec;

        control::ProfiledPID profiled_lateral_pid(profiled_constants);
        profiled_lateral_pid.reset(0.0); // Start at 0 distance traveled

        // Reset angular PID for heading correction
        angular_pid.reset();

        reset_telemetry_accumulators();

        while ((pros::millis() - start_time) < timeout.to_millis_uint())
        {
            estimation::Pose current_pose = get_pose();

            // Calculate distance traveled using vector projection
            double dx = current_pose.x() - initial_x;
            double dy = current_pose.y() - initial_y;
            double distance_traveled_raw = dx * std::cos(start_heading) + dy * std::sin(start_heading);

            // Compute profiled PID output for lateral control
            double lateral_output = profiled_lateral_pid.compute(distance_traveled_raw, target_distance.inches, dt);

            // Add feedforward for lateral motion
            double target_velocity = profiled_lateral_pid.get_setpoint_velocity();
            double target_acceleration = profiled_lateral_pid.get_setpoint().acceleration;

            // Feedforward with deadband to avoid static friction at zero velocity
            double ff_sign = (std::abs(target_velocity) < 0.01) ? 0.0 : math::sgn(target_velocity);
            double lateral_ff = config_.lateral_kS * ff_sign +
                                config_.lateral_kV * target_velocity +
                                config_.lateral_kA * target_acceleration;

            lateral_output += lateral_ff;

            // Calculate angular error and correction (maintain starting heading)
            double angular_error = start_heading - current_pose.theta();
            angular_error = math::normalize_angle(angular_error);
            double angular_output = angular_pid.compute(angular_error, dt);

            // Clamp outputs
            lateral_output = std::clamp(lateral_output, -12.0, 12.0);
            angular_output = std::clamp(angular_output, -12.0, 12.0);

            // Calculate motor voltages
            units::Voltage left_voltage = units::Voltage::from_volts(lateral_output + angular_output);
            units::Voltage right_voltage = units::Voltage::from_volts(lateral_output - angular_output);

            // Check settlement
            if (profiled_lateral_pid.at_goal() && std::abs(angular_error) <= settlement_config_.angular_threshold.value)
            {
                settle_count++;
                if (settle_count >= settlement_config_.settle_count_required)
                {
                    update_settlement_telemetry(true, settle_count, SettlementReason::WITHIN_THRESHOLD, start_time);
                    left_motors->brake();
                    right_motors->brake();
                    break;
                }
            }
            else
            {
                settle_count = 0;
            }

            // Update telemetry
            units::Distance distance_traveled = units::Distance::from_inches(distance_traveled_raw);
            units::Distance lateral_error = target_distance - distance_traveled;
            update_lateral_telemetry(lateral_error, lateral_output, target_distance, distance_traveled, dt);
            update_angular_telemetry(angular_error, angular_output, start_heading, current_pose.theta(), dt);
            update_pose_telemetry(current_pose);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, settle_count, SettlementReason::NOT_SETTLED, start_time);

            // Send power to motors
            move_left_motors(left_voltage);
            move_right_motors(right_voltage);
            telemetry.swap();
            pros::delay(10);
        }

        // Check if we timed out
        bool timed_out = (pros::millis() - start_time) >= timeout.to_millis_uint();
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            auto &data = telemetry.get_write_buffer();
            if (timed_out && !data.is_settled)
            {
                data.settlement_reason = SettlementReason::TIMEOUT;
                data.time_to_settle = units::Time::from_millis(pros::millis() - start_time);
            }
        }

        left_motors->brake();
        right_motors->brake();
    }
}