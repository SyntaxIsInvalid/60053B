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
        units::Length distance,
        units::Velocity max_velocity,
        double max_acceleration,
        units::Time timeout,
        double heading_tolerance)
    {
        // Get current pose from odometry (REP-103 body frame)
        estimation::Pose current_body = get_pose();

        // Convert current pose to math frame
        units::Length start_x_math, start_y_math;
        units::Angle start_theta_math;

        // Create BodyPose from Pose members
        math::BodyPose body_pose{current_body.x, current_body.y, current_body.theta};
        math::body_to_math_frame(body_pose, start_x_math, start_y_math, start_theta_math);

        // Calculate end pose in math frame
        // Move in the direction of current heading
        double end_x_math = start_x_math.to_inches() + distance.to_inches() * std::cos(start_theta_math.to_radians());
        double end_y_math = start_y_math.to_inches() + distance.to_inches() * std::sin(start_theta_math.to_radians());
        double end_theta_math = start_theta_math.to_radians(); // Heading stays the same

        // Create path segment in math frame
        path::Pose start_pose(start_x_math.to_inches(), start_y_math.to_inches(), start_theta_math.to_radians());
        path::Pose end_pose(end_x_math, end_y_math, end_theta_math);

        path::StraightSegment segment(start_pose, end_pose, heading_tolerance);

        // Configure trajectory follower
        trajectory::FollowerConfig config;
        config.max_velocity = max_velocity;
        config.max_acceleration = units::Acceleration::from_mps2(max_acceleration);
        config.timeout = timeout;
        config.ramsete_constants = config_.ramsete_constants;

        // Execute the trajectory
        path_follower_->follow_segment(&segment, config);
    }

    void Chassis::turn_to_heading_profiled_pid(
        units::Angle target_heading,
        units::AngularVelocity max_angular_velocity,
        units::AngularAcceleration max_angular_acceleration,
        units::Time timeout)
    {
        // Convert to SI doubles at the boundary
        double target_rad = target_heading.to_radians();
        double max_vel_rad_s = max_angular_velocity.to_rad_per_sec();
        double max_accel_rad_s2 = max_angular_acceleration.to_rad_per_sec2();

        std::uint32_t start_time = pros::millis();
        const double dt = 0.01;

        // Get initial heading
        units::Angle initial_heading = get_heading();
        double initial_rad = initial_heading.to_radians();
        double last_wrapped_rad = initial_rad;
        double cumulative_unwrapped_rad = initial_rad;

        // Unwrap target to be close to initial
        double angle_diff = target_rad - initial_rad;
        angle_diff = math::normalize_angle(angle_diff);
        double unwrapped_target = initial_rad + angle_diff;

        // Create ProfiledPID with doubles
        control::ProfiledPIDConstants profiled_constants;
        profiled_constants.pid_constants = config_.profiled_turn_pid_constants;
        profiled_constants.max_velocity = max_vel_rad_s;
        profiled_constants.max_acceleration = max_accel_rad_s2;
        profiled_constants.position_tolerance = settlement_config_.angular_threshold.to_radians();
        profiled_constants.velocity_tolerance = settlement_config_.angular_velocity_threshold.to_rad_per_sec();

        control::ProfiledPID profiled_pid(profiled_constants);
        profiled_pid.reset(cumulative_unwrapped_rad);

        reset_telemetry_accumulators();

        int settle_count = 0;
        const int REQUIRED_SETTLE_COUNT = 3;

        while ((pros::millis() - start_time) < timeout.to_milliseconds())
        {
            // Get current wrapped heading
            units::Angle current_heading = get_heading();
            double current_wrapped_rad = current_heading.to_radians();

            // Calculate the delta and unwrap it
            double delta_rad = current_wrapped_rad - last_wrapped_rad;
            delta_rad = math::normalize_angle(delta_rad);
            cumulative_unwrapped_rad += delta_rad;
            last_wrapped_rad = current_wrapped_rad;

            estimation::Pose current_pose = get_pose();

            // Compute profiled PID output (all doubles now)
            double angular_output = profiled_pid.compute(
                cumulative_unwrapped_rad,
                unwrapped_target,
                dt);

            // Calculate feedforward (doubles from ProfiledPID)
            double target_velocity = profiled_pid.get_setpoint_velocity();
            double target_acceleration = profiled_pid.get_setpoint().acceleration;

            double ff_sign = (std::abs(target_velocity) < 0.01) ? 0.0 : math::sgn(target_velocity);

            double ff = config_.turn_in_place_kS * ff_sign +
                        config_.turn_in_place_kV * target_velocity +
                        config_.turn_in_place_kA * target_acceleration;

            angular_output += ff;
            angular_output = std::clamp(angular_output, -12.0, 12.0);

            // Calculate wrapped error for telemetry
            double angular_error_rad = unwrapped_target - cumulative_unwrapped_rad;
            angular_error_rad = math::normalize_angle(angular_error_rad);

            // Check settlement
            if (profiled_pid.at_goal())
            {
                settle_count++;
                if (settle_count >= REQUIRED_SETTLE_COUNT)
                {
                    update_settlement_telemetry(true, settle_count,
                                                telemetry::SettlementReason::WITHIN_THRESHOLD, start_time);
                    left_motors->brake();
                    right_motors->brake();
                    break;
                }
            }
            else
            {
                settle_count = 0;
            }

            // Convert back to units for motor control
            units::Voltage left_voltage = units::Voltage::from_volts(-angular_output);
            units::Voltage right_voltage = units::Voltage::from_volts(angular_output);

            update_angular_telemetry(angular_error_rad, angular_output,
                                     unwrapped_target, cumulative_unwrapped_rad, dt);
            update_pose_telemetry(current_pose);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, settle_count,
                                        telemetry::SettlementReason::NOT_SETTLED, start_time);

            move_left_motors(left_voltage);
            move_right_motors(right_voltage);
            telemetry::g_telemetry.swap();
            pros::delay(10);
        }

        // Timeout handling
        left_motors->brake();
        right_motors->brake();
    }
    /*
    void Chassis::drive_straight_profiled_pid(
        units::Length target_distance,
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
        double start_heading = start_pose.theta.to_radians();
        double initial_x = start_pose.x.to_inches();
        double initial_y = start_pose.y.to_inches();
        int settle_count = 0;

        // Create ProfiledPID for lateral control
        control::ProfiledPIDConstants profiled_constants;
        profiled_constants.pid_constants = config_.profiled_lateral_pid_constants;
        profiled_constants.max_velocity = units::Velocity::from_ips(max_velocity_inches_per_sec);
        profiled_constants.max_acceleration = units::Acceleration::from_mps2(max_acceleration_inches_per_sec2 * units::constants::IPS_TO_MPS);
        profiled_constants.position_tolerance = settlement_config_.position_threshold;
        profiled_constants.velocity_tolerance = settlement_config_.linear_velocity_threshold;

        control::ProfiledPID profiled_lateral_pid(profiled_constants);
        profiled_lateral_pid.reset(units::Length::from_inches(0.0)); // Start at 0 distance traveled

        // Reset angular PID for heading correction
        angular_pid.reset();

        reset_telemetry_accumulators();

        while ((pros::millis() - start_time) < timeout.to_milliseconds())
        {
            estimation::Pose current_pose = get_pose();

            // Calculate distance traveled using vector projection
            double dx = current_pose.x.to_inches() - initial_x;
            double dy = current_pose.y.to_inches() - initial_y;
            double distance_traveled_raw = dx * std::cos(start_heading) + dy * std::sin(start_heading);

            // Compute profiled PID output for lateral control (passing units::Length)
            double lateral_output = profiled_lateral_pid.compute(
                units::Length::from_inches(distance_traveled_raw),
                target_distance,
                units::Time::from_seconds(dt));

            // Add feedforward for lateral motion
            units::Velocity target_velocity = profiled_lateral_pid.get_setpoint_velocity();
            double target_acceleration = profiled_lateral_pid.get_setpoint().acceleration.to_mps2();

            // Feedforward with deadband to avoid static friction at zero velocity
            double ff_sign = (std::abs(target_velocity.to_ips()) < 0.01) ? 0.0 : math::sgn(target_velocity.to_ips());
            double lateral_ff = config_.lateral_kS * ff_sign +
                                config_.lateral_kV * target_velocity.to_ips() +
                                config_.lateral_kA * target_acceleration;

            lateral_output += lateral_ff;

            // Calculate angular error and correction (maintain starting heading)
            double angular_error = start_heading - current_pose.theta.to_radians();
            angular_error = math::normalize_angle(angular_error);
            double angular_output = angular_pid.compute(angular_error, dt);

            // Clamp outputs
            lateral_output = std::clamp(lateral_output, -12.0, 12.0);
            angular_output = std::clamp(angular_output, -12.0, 12.0);

            // Calculate motor voltages
            units::Voltage left_voltage = units::Voltage::from_volts(lateral_output + angular_output);
            units::Voltage right_voltage = units::Voltage::from_volts(lateral_output - angular_output);

            // Check settlement
            if (profiled_lateral_pid.at_goal() && std::abs(angular_error) <= settlement_config_.angular_threshold.to_radians())
            {
                settle_count++;
                if (settle_count >= settlement_config_.settle_count_required)
                {
                    update_settlement_telemetry(true, settle_count, telemetry::SettlementReason::WITHIN_THRESHOLD, start_time);
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
            units::Length distance_traveled = units::Length::from_inches(distance_traveled_raw);
            units::Length lateral_error = units::Length::from_inches(target_distance.to_inches() - distance_traveled_raw);
            update_lateral_telemetry(lateral_error, lateral_output, target_distance, distance_traveled, dt);
            update_angular_telemetry(angular_error, angular_output, start_heading, current_pose.theta.to_radians(), dt);
            update_pose_telemetry(current_pose);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, settle_count, telemetry::SettlementReason::NOT_SETTLED, start_time);

            // Send power to motors
            move_left_motors(left_voltage);
            move_right_motors(right_voltage);
            telemetry::g_telemetry.swap();
            pros::delay(10);
        }

        // Check if we timed out
        bool timed_out = (pros::millis() - start_time) >= timeout.to_milliseconds();
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            auto &data = telemetry::g_telemetry.get_write_buffer();
            if (timed_out && !data.is_settled)
            {
                data.settlement_reason = telemetry::SettlementReason::TIMEOUT;
                data.time_to_settle = units::Time::from_milliseconds(pros::millis() - start_time);
            }
        }

        left_motors->brake();
        right_motors->brake();
    }
        */
}