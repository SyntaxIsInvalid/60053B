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
#include "abclib/path/quintic_hermite_segment.hpp"
using namespace abclib;

namespace abclib::hardware
{
    void Chassis::move_straight_profiled(
        units::Length distance,
        units::Velocity max_velocity,
        units::Acceleration max_acceleration,
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
        config.max_acceleration = max_acceleration; // Already a unit now
        config.timeout = timeout;
        config.ramsete_constants = config_.controllers.ramsete;

        // Execute the trajectory
        path_follower_->follow_segment(&segment, config);
    }

    void Chassis::turn_to_heading_profiled(
        units::Angle target_heading,
        units::AngularVelocity max_body_angular_velocity,
        units::AngularAcceleration max_body_angular_acceleration,
        units::Time timeout)
    {
        // 1. Get current pose from odometry
        estimation::Pose current_pose = get_pose();
        double current_heading_rad = current_pose.theta.to_radians();
        double target_heading_rad = target_heading.to_radians();

        // 2. Create TurnInPlaceSegment in math frame
        path::Pose start_pose(current_pose.x.to_inches(), current_pose.y.to_inches(), current_heading_rad);
        path::TurnInPlaceSegment turn_segment(start_pose, target_heading_rad, track_width);

        // 3. Convert angular velocity/acceleration to linear (wheel velocity)
        // For turn-in-place: v_wheel = ω_body * r, where r = track_width / 2
        double turning_radius = track_width.to_inches() / 2.0;
        double max_angular_vel_rad_per_sec = max_body_angular_velocity.to_rad_per_sec();
        double max_angular_accel_rad_per_sec2 = max_body_angular_acceleration.to_rad_per_sec2();

        double max_wheel_linear_velocity = max_angular_vel_rad_per_sec * turning_radius;
        double max_wheel_linear_accel = max_angular_accel_rad_per_sec2 * turning_radius;

        // 4. Configure follower with converted linear velocities
        trajectory::FollowerConfig config;
        config.max_velocity = units::Velocity::from_ips(max_wheel_linear_velocity);
        config.max_acceleration = units::Acceleration::from_mps2(max_wheel_linear_accel);
        config.timeout = timeout;
        config.ramsete_constants = config_.controllers.ramsete;
        config.turn_kP = 0.35; // Can make this configurable via ChassisConfig later

        // 5. Let the path follower handle everything!
        path_follower_->follow_segment(&turn_segment, config);
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
        profiled_constants.pid_constants = config_.controllers.profiled_turn_pid;
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

            double ff = config_.controllers.turn_in_place_kS * ff_sign +
                        config_.controllers.turn_in_place_kV * target_velocity +
                        config_.controllers.turn_in_place_kA * target_acceleration;

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

    void Chassis::move_straight_profiled_pid(
        units::Length target_distance,
        units::Velocity max_velocity,
        units::Acceleration max_acceleration,
        units::Time timeout,
        bool reset_position)
    {
        std::uint32_t start_time = pros::millis();
        const double dt = 0.01;

        // Get initial position
        estimation::Pose initial_pose = get_pose();
        units::Length initial_x = initial_pose.x;
        units::Length initial_y = initial_pose.y;
        units::Angle initial_heading = initial_pose.theta;

        if (reset_position)
        {
            reset_chassis_position();
            initial_x = units::Length::from_inches(0);
            initial_y = units::Length::from_inches(0);
            initial_heading = units::Angle::from_radians(0);
        }

        // Create ProfiledPID for lateral control
        control::ProfiledPIDConstants lateral_profiled_constants;
        lateral_profiled_constants.pid_constants = config_.controllers.lateral_pid;
        lateral_profiled_constants.max_velocity = max_velocity.to_ips();
        lateral_profiled_constants.max_acceleration = max_acceleration.to_mps2();
        lateral_profiled_constants.position_tolerance = settlement_config_.position_threshold.to_inches();
        lateral_profiled_constants.velocity_tolerance = settlement_config_.linear_velocity_threshold.to_ips();

        control::ProfiledPID lateral_profiled_pid(lateral_profiled_constants);
        lateral_profiled_pid.reset(0.0); // Start at 0 distance traveled

        reset_telemetry_accumulators();

        int settle_count = 0;
        const int REQUIRED_SETTLE_COUNT = 3;

        while ((pros::millis() - start_time) < timeout.to_milliseconds())
        {
            estimation::Pose current_pose = get_pose();

            // Calculate distance traveled along the initial heading direction
            double dx = current_pose.x.to_inches() - initial_x.to_inches();
            double dy = current_pose.y.to_inches() - initial_y.to_inches();
            double distance_traveled = dx * std::cos(initial_heading.to_radians()) +
                                       dy * std::sin(initial_heading.to_radians());

            // Compute lateral profiled PID output
            double lateral_output = lateral_profiled_pid.compute(
                distance_traveled,
                target_distance.to_inches(),
                dt);

            // Get setpoint for feedforward
            double target_velocity_ips = lateral_profiled_pid.get_setpoint_velocity();
            double target_acceleration = lateral_profiled_pid.get_setpoint().acceleration;

            // Calculate feedforward
            double ff_sign = (std::abs(target_velocity_ips) < 0.01) ? 0.0 : math::sgn(target_velocity_ips);
            double ff = config_.controllers.lateral_kS * ff_sign +
                        config_.controllers.lateral_kV * target_velocity_ips +
                        config_.controllers.lateral_kA * target_acceleration;

            lateral_output += ff;

            // Clamp output
            lateral_output = std::clamp(lateral_output, -12.0, 12.0);

            // Apply same output to both sides (no angular correction)
            double left_voltage = lateral_output;
            double right_voltage = lateral_output;

            // Check settlement
            if (lateral_profiled_pid.at_goal())
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

            // Update telemetry
            units::Length position_error = units::Length::from_inches(target_distance.to_inches() - distance_traveled);
            units::Length current_distance = units::Length::from_inches(distance_traveled);

            update_lateral_telemetry(position_error, lateral_output,
                                     target_distance, current_distance, dt);
            update_pose_telemetry(current_pose);
            update_motor_voltage_telemetry(units::Voltage::from_volts(left_voltage),
                                           units::Voltage::from_volts(right_voltage));
            update_settlement_telemetry(false, settle_count,
                                        telemetry::SettlementReason::NOT_SETTLED, start_time);

            // Apply voltages
            move_left_motors(units::Voltage::from_volts(left_voltage));
            move_right_motors(units::Voltage::from_volts(right_voltage));

            telemetry::g_telemetry.swap();
            pros::delay(10);
        }

        // Timeout handling
        left_motors->brake();
        right_motors->brake();

        // Final telemetry update
        update_settlement_telemetry(false, settle_count,
                                    telemetry::SettlementReason::TIMEOUT, start_time);
    }

    void Chassis::move_to_pose_profiled(
        units::Length target_x,
        units::Length target_y,
        units::Angle target_heading,
        units::Velocity max_velocity,
        units::Acceleration max_acceleration,
        units::Time timeout)
    {
        // 1. Get current pose from odometry (REP-103 body frame)
        estimation::Pose current_body = get_pose();

        // 2. Convert current pose to math frame
        units::Length start_x_math, start_y_math;
        units::Angle start_theta_math;

        math::BodyPose body_pose{current_body.x, current_body.y, current_body.theta};
        math::body_to_math_frame(body_pose, start_x_math, start_y_math, start_theta_math);

        // 3. Convert target pose to math frame (assuming target is given in body frame)
        units::Length target_x_math, target_y_math;
        units::Angle target_theta_math;

        math::BodyPose target_body_pose{target_x, target_y, target_heading};
        math::body_to_math_frame(target_body_pose, target_x_math, target_y_math, target_theta_math);

        // 4. Create quintic Hermite spline in math frame
        path::Pose start_pose(start_x_math.to_inches(), start_y_math.to_inches(), start_theta_math.to_radians());
        path::Pose end_pose(target_x_math.to_inches(), target_y_math.to_inches(), target_theta_math.to_radians());

        // Use heuristic constructor (1.2x distance scaling)
        path::QuinticHermiteSegment segment(start_pose, end_pose);

        // 5. Configure trajectory follower
        trajectory::FollowerConfig config;
        config.max_velocity = max_velocity;
        config.max_acceleration = max_acceleration;
        config.timeout = timeout;
        config.ramsete_constants = config_.controllers.ramsete;

        // 6. Execute the trajectory using path follower
        path_follower_->follow_segment(&segment, config);
    }

}