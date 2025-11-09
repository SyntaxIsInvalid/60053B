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
        double max_angular_velocity_rad_per_sec,
        double max_angular_acceleration_rad_per_sec2,
        units::Time timeout)
    {
        std::uint32_t start_time = pros::millis();
        const double dt = 0.01;

        // Convert target heading to radians
        units::Radians target_heading_rad = target_heading.to_radians();
        double target_rad = target_heading_rad.value;

        // Get initial heading
        units::BodyHeading initial_heading = get_heading();
        double initial_rad = initial_heading.angle.value;
        double last_wrapped_rad = initial_rad;
        double cumulative_unwrapped_rad = initial_rad; // Track unwrapped position

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
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            telemetry.max_angular_error = units::Radians(0);
            telemetry.cumulative_angular_error = units::Radians(0);
        }

        while ((pros::millis() - start_time) < timeout.to_millis_uint())
        {
            // Get current wrapped heading
            units::BodyHeading current_heading = get_heading();
            double current_wrapped_rad = current_heading.angle.value;

            // Calculate the delta and unwrap it
            double delta_rad = current_wrapped_rad - last_wrapped_rad;
            delta_rad = math::normalize_angle(delta_rad); // Handle wrapping
            cumulative_unwrapped_rad += delta_rad;
            last_wrapped_rad = current_wrapped_rad;

            estimation::Pose current_pose = get_pose();

            // Compute profiled PID output using unwrapped measurement
            double angular_output = profiled_pid.compute(cumulative_unwrapped_rad, unwrapped_target, dt);

            double target_velocity = profiled_pid.get_setpoint_velocity();
            double target_acceleration = profiled_pid.get_setpoint().acceleration;
            double ff = config_.turn_in_place_kS * math::sgn(target_velocity) + config_.turn_in_place_kV * target_velocity + config_.turn_in_place_kA * target_acceleration;

            angular_output += ff;
            // Clamp output
            angular_output = std::clamp(angular_output, -12.0, 12.0);

            // Calculate wrapped error for telemetry
            double angular_error_rad = unwrapped_target - cumulative_unwrapped_rad;
            angular_error_rad = math::normalize_angle(angular_error_rad);
            units::Radians angular_error(angular_error_rad);
            // Check settlement
            if (profiled_pid.at_goal())
            {
                {
                    std::lock_guard<pros::Mutex> lock(telemetry_mutex);
                    telemetry.is_settled = true;
                    telemetry.settlement_reason = SettlementReason::WITHIN_THRESHOLD;
                    telemetry.time_to_settle = units::Time::from_millis(pros::millis() - start_time);
                }
                left_motors->brake();
                right_motors->brake();
                break;
            }

            // Motor voltages
            units::Voltage left_voltage = units::Voltage(-angular_output);
            units::Voltage right_voltage = units::Voltage(angular_output);

            // Update telemetry
            {
                std::lock_guard<pros::Mutex> lock(telemetry_mutex);

                telemetry.angular_error = angular_error;
                telemetry.angular_output = units::Voltage(angular_output);
                telemetry.angular_target = units::Radians(unwrapped_target);
                telemetry.angular_actual = units::Radians(cumulative_unwrapped_rad);
                telemetry.angular_p_term = profiled_pid.get_pid().get_p_term();
                telemetry.angular_i_term = profiled_pid.get_pid().get_i_term();
                telemetry.angular_d_term = profiled_pid.get_pid().get_d_term();

                telemetry.pose = current_pose.pose;
                telemetry.pose_v = current_pose.v;
                telemetry.pose_omega = current_pose.omega;

                telemetry.is_settled = false;
                telemetry.settlement_reason = SettlementReason::NOT_SETTLED;

                telemetry.max_angular_error = units::Radians(
                    std::max(telemetry.max_angular_error.value, std::abs(angular_error_rad)));
                telemetry.cumulative_angular_error = units::Radians(
                    telemetry.cumulative_angular_error.value + std::abs(angular_error_rad) * dt);

                telemetry.left_motor_voltage = left_voltage;
                telemetry.right_motor_voltage = right_voltage;
            }

            move_left_motors(left_voltage);
            move_right_motors(right_voltage);

            pros::delay(10);
        }

        // Timeout check
        bool timed_out = (pros::millis() - start_time) >= timeout.to_millis_uint();
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            if (timed_out && !telemetry.is_settled)
            {
                telemetry.settlement_reason = SettlementReason::TIMEOUT;
                telemetry.time_to_settle = units::Time::from_millis(pros::millis() - start_time);
            }
        }

        left_motors->brake();
        right_motors->brake();
    }

}