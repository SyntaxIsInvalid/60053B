#include "abclib/hardware/chassis.hpp"
#include "api.h"
#include <mutex>
#include "abclib/path/straight_segment.hpp"
#include "abclib/path/turn_in_place_segment.hpp"
#include <fstream>
#include "abclib/kinematics/differential_drive.hpp"
#include "abclib/measurement/wheel_measurement_models.hpp"
#include "abclib/measurement/imu_measurement_model.hpp"
#include "abclib/math/coordinate_frames.hpp"
#include "abclib/path/straight_segment.hpp"
#include "abclib/telemetry/logger.hpp"
#include "abclib/trajectory/trajectory.hpp"
#include "abclib/math/point.hpp"
using namespace abclib;

namespace abclib::hardware
{
    void Chassis::turn_to_heading(units::Angle target_heading_corner,
                                  units::Time timeout,
                                  units::Voltage angular_max,
                                  bool reset_position)
    {
        std::uint32_t start_time = pros::millis();
        int settle_count = 0;
        const double dt = 0.01;

        // Convert target heading from corner frame to standard frame
        units::Angle target_heading_standard;
        if (alliance_ == field::Alliance::RED)
        {
            // Red: corner 0° (north) = standard 90°
            target_heading_standard = target_heading_corner + units::Angle::from_radians(M_PI_2);
        }
        else // BLUE
        {
            // Blue: corner 0° (south) = standard -90°
            target_heading_standard = target_heading_corner + units::Angle::from_radians(-M_PI_2);
        }

        double target_heading_rad = target_heading_standard.to_radians();

        if (reset_position)
        {
            imu->set_heading(0);
        }

        angular_pid.reset();
        reset_telemetry_accumulators();

        while ((pros::millis() - start_time) < timeout.to_milliseconds())
        {
            // Get current pose from estimator (standard frame)
            estimation::Pose current_pose_standard = get_pose_standard();
            double current_heading_rad = current_pose_standard.theta_rad();

            // Calculate angular error in standard frame
            double angular_error_rad = target_heading_rad - current_heading_rad;
            angular_error_rad = math::normalize_angle(angular_error_rad);

            // Check settlement
            if (check_angular_settlement(
                    units::Angle::from_radians(angular_error_rad),
                    current_pose_standard.omega,
                    settle_count))
            {
                update_settlement_telemetry(true, settle_count,
                                            telemetry::SettlementReason::WITHIN_THRESHOLD, start_time);
                left_motors->brake();
                right_motors->brake();
                break;
            }

            // Calculate PID output
            double angular_output = angular_pid.compute(angular_error_rad, dt);

            // Simple clamp
            angular_output = std::clamp(angular_output, -angular_max.to_volts(), angular_max.to_volts());

            // Calculate motor voltages (opposite for turning)
            units::Voltage left_voltage = units::Voltage::from_volts(-angular_output);
            units::Voltage right_voltage = units::Voltage::from_volts(angular_output);

            // Get corner frame pose for telemetry
            estimation::Pose current_pose_corner = get_pose_alliance_corner();

            // Update telemetry
            update_angular_telemetry(angular_error_rad, angular_output,
                                     target_heading_rad, current_heading_rad, dt);
            update_pose_telemetry(current_pose_corner);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, settle_count,
                                        telemetry::SettlementReason::NOT_SETTLED, start_time);

            // Apply to motors
            move_left_motors(left_voltage);
            move_right_motors(right_voltage);
            telemetry::g_telemetry.swap();

            pros::delay(10);
        }

        // Check timeout
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

    void Chassis::turn_relative(units::Angle angle_delta,
                                units::Time timeout,
                                units::Voltage angular_max)
    {
        // Get current heading
        units::Angle current_heading = get_heading();

        // Calculate target heading by adding the delta
        units::Angle target_heading = units::Angle::from_radians(
            current_heading.to_radians() + angle_delta.to_radians());

        // Call the absolute turning function
        turn_to_heading(target_heading, timeout, angular_max, false);
    }

    void Chassis::drive_straight_relative(units::Length target_distance,
                                          units::Time timeout,
                                          units::Voltage lateral_min,
                                          units::Voltage lateral_max,
                                          units::Voltage angular_min,
                                          units::Voltage angular_max,
                                          bool reset_position)
    {
        std::uint32_t start_time = pros::millis();
        const double dt = 0.01;

        if (reset_position)
        {
            reset_chassis_position();
        }

        // Get start pose in standard frame
        estimation::Pose start_pose = get_pose_standard();
        double start_heading = start_pose.theta_rad();
        double initial_x = start_pose.x_inches();
        double initial_y = start_pose.y_inches();
        int settle_count = 0;

        lateral_pid.reset();
        angular_pid.reset();
        reset_telemetry_accumulators();

        while ((pros::millis() - start_time) < timeout.to_milliseconds())
        {
            // Get current pose in standard frame
            estimation::Pose current_pose_standard = get_pose_standard();

            // Calculate distance traveled using vector projection
            double dx = current_pose_standard.x_inches() - initial_x;
            double dy = current_pose_standard.y_inches() - initial_y;
            double distance_traveled_raw = dx * std::cos(start_heading) + dy * std::sin(start_heading);
            units::Length distance_traveled = units::Length::from_inches(distance_traveled_raw);

            // Calculate lateral error
            units::Length lateral_error = target_distance - distance_traveled;

            // Calculate angular error (maintain start heading)
            double angular_error_rad = start_heading - current_pose_standard.theta_rad();
            angular_error_rad = math::normalize_angle(angular_error_rad);

            // Check settlement
            if (check_linear_settlement(lateral_error, current_pose_standard.v, settle_count))
            {
                update_settlement_telemetry(true, settle_count,
                                            telemetry::SettlementReason::WITHIN_THRESHOLD, start_time);
                left_motors->brake();
                right_motors->brake();
                break;
            }

            // Calculate PID outputs
            double lateral_output = lateral_pid.compute(lateral_error.to_inches(), dt);
            double angular_output = angular_pid.compute(angular_error_rad, dt);

            // Simple clamp
            lateral_output = std::clamp(lateral_output, -lateral_max.to_volts(), lateral_max.to_volts());
            angular_output = std::clamp(angular_output, -angular_max.to_volts(), angular_max.to_volts());

            // Calculate motor voltages
            units::Voltage left_voltage = units::Voltage::from_volts(lateral_output - angular_output);
            units::Voltage right_voltage = units::Voltage::from_volts(lateral_output + angular_output);

            // Get corner frame pose for telemetry
            estimation::Pose current_pose_corner = get_pose_alliance_corner();

            // Update telemetry
            update_lateral_telemetry(lateral_error, lateral_output, target_distance, distance_traveled, dt);
            update_angular_telemetry(angular_error_rad, angular_output, start_heading,
                                     current_pose_standard.theta_rad(), dt);
            update_pose_telemetry(current_pose_corner);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, settle_count,
                                        telemetry::SettlementReason::NOT_SETTLED, start_time);

            // Apply to motors
            move_left_motors(left_voltage);
            move_right_motors(right_voltage);
            telemetry::g_telemetry.swap();

            pros::delay(10);
        }

        // Check timeout
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

    void Chassis::boomerang_move_to_pose(
        units::Length target_x,
        units::Length target_y,
        units::Angle target_heading,
        const BoomerangConfig &config)
    {
        std::uint32_t start_time = pros::millis();
        const double dt = 0.01;
        int drive_settle_count = 0;
        int turn_settle_count = 0;
        bool passed = false;

        // --- Convert target from corner frame to standard frame ---
        // (same pattern as set_pose_alliance_corner)
        estimation::Pose target_corner_pose;
        target_corner_pose.se2 = math::SE2(
            target_x.to_inches(),
            target_y.to_inches(),
            target_heading.to_radians());
        target_corner_pose.v = units::Velocity::from_ips(0);
        target_corner_pose.omega = units::AngularVelocity::from_rad_per_sec(0);

        estimation::Pose target_std = field::alliance_corner_to_standard(
            target_corner_pose, alliance_, config_.field_config);

        const double tx = target_std.x_inches();
        const double ty = target_std.y_inches();
        const double target_theta = target_std.theta_rad();

        // Robot forward direction at target in world frame
        // (+y forward convention: forward = (cos theta, sin theta))
        const double etx = std::cos(target_theta);
        const double ety = std::sin(target_theta);

        // --- Determine direction sign ---
        double sign = 1.0;
        {
            auto pose = get_pose_standard();
            if (config.direction == BoomerangConfig::Direction::REVERSE)
            {
                sign = -1.0;
            }
            else if (config.direction == BoomerangConfig::Direction::FLEXIBLE)
            {
                // If robot is already past the target (end tangent points back at robot),
                // reverse approach
                double rpx = pose.x_inches() - tx;
                double rpy = pose.y_inches() - ty;
                if (etx * rpx + ety * rpy > 0.0)
                    sign = -1.0;
            }
        }

        lateral_pid.reset();
        angular_pid.reset();
        reset_telemetry_accumulators();

        while ((pros::millis() - start_time) < config.timeout.to_milliseconds())
        {
            auto pose = get_pose_standard();
            double px = pose.x_inches();
            double py = pose.y_inches();
            double theta = pose.theta_rad();

            // --- Carrot point ---
            double dx = tx - px;
            double dy = ty - py;
            double d = std::sqrt(dx * dx + dy * dy);

            // Carrot slides from behind target toward target as d shrinks
            double carrot_x = tx - d * config.lead_distance * sign * etx;
            double carrot_y = ty - d * config.lead_distance * sign * ety;

            // --- Rotate carrot vector into body frame ---
            // World frame: +y north, +x east
            // Body frame:  +y forward, +x right lateral
            // body_fwd = dot(ec, robot_forward) = ecx*cos(theta) + ecy*sin(theta)
            // body_lat = dot(ec, robot_right)   = ecx*sin(theta) - ecy*cos(theta)
            double ecx = carrot_x - px;
            double ecy = carrot_y - py;
            double body_fwd = ecx * std::cos(theta) + ecy * std::sin(theta);
            double body_lat = ecx * std::sin(theta) - ecy * std::cos(theta);

            // --- Heading error (CCW positive, same convention as turn_to_heading) ---
            // Carrot to the right (body_lat > 0) → need CW turn → negative error
            double carrot_heading_error = -std::atan2(body_lat, body_fwd);
            double cosine_scale = std::cos(carrot_heading_error);

            // --- Drive error (blended distance: carrot distance + raw distance) ---
            double carrot_dist = std::sqrt(body_lat * body_lat + body_fwd * body_fwd);
            double drive_error = sign * std::sqrt((carrot_dist * carrot_dist + 2.0 * d * d) / 3.0) * (cosine_scale >= 0.0 ? 1.0 : -1.0);

            // --- Passing detection ---
            // Project target vector onto end tangent to get longitudinal component
            double long_error = dx * etx + dy * ety;
            double lat_dist = std::abs(-dx * ety + dy * etx);
            double pass_thresh = settlement_config_.position_threshold.to_inches() * 4.0;
            passed |= (sign * long_error <= 0.0) && (lat_dist < pass_thresh);

            bool is_settling = std::abs(drive_error) < settlement_config_.position_threshold.to_inches();

            // --- Switch heading target when near/past the goal ---
            double heading_error_rad;
            if (d < pass_thresh || passed)
            {
                if (is_settling)
                {
                    // Align to final heading
                    heading_error_rad = math::normalize_angle(target_theta - theta);
                }
                else
                {
                    // Clamp to ±90° to prevent a 180° spin instead of small correction
                    heading_error_rad = std::clamp(carrot_heading_error, -M_PI_2, M_PI_2);
                }
            }
            else
            {
                heading_error_rad = carrot_heading_error;
            }

            // --- PID ---
            double drive_output = lateral_pid.compute(drive_error, dt);
            double turn_output = angular_pid.compute(heading_error_rad, dt);

            // --- Lateral correction (RAMSETE-inspired) ---
            // Adds extra turn proportional to lateral error when not yet settling
            // Dampened by sinc so it has no effect if already facing the carrot
            if (!is_settling)
            {
                double sinc_val = (std::abs(carrot_heading_error) > 1e-6)
                                      ? std::sin(carrot_heading_error) / carrot_heading_error
                                      : 1.0;

                // body_lat > 0 (carrot right) → need CW → subtract from CCW-positive output
                turn_output -= drive_output * config.lateral_correction * body_lat * sinc_val / track_width.to_inches();
            }

            // --- Clamp outputs ---
            drive_output = std::clamp(drive_output,
                                      -config.drive_max.to_volts(),
                                      config.drive_max.to_volts());
            turn_output = std::clamp(turn_output,
                                     -config.turn_max.to_volts(),
                                     config.turn_max.to_volts());

            // --- Settlement check ---
            // Drive: distance to target
            // Turn:  final heading error (not carrot error)
            units::Length dist_error = units::Length::from_inches(d);
            units::Angle head_error = units::Angle::from_radians(
                math::normalize_angle(target_theta - theta));

            bool drive_settled = check_linear_settlement(dist_error, pose.v, drive_settle_count);
            bool turn_settled = check_angular_settlement(head_error, pose.omega, turn_settle_count);

            if (drive_settled && turn_settled)
            {
                update_settlement_telemetry(true, drive_settle_count,
                                            telemetry::SettlementReason::WITHIN_THRESHOLD,
                                            start_time);
                left_motors->brake();
                right_motors->brake();
                break;
            }

            // --- Motor voltages ---
            // Same convention as turn_to_heading:
            // left = drive - turn, right = drive + turn
            // (positive turn_output = CCW = right motor gets more)
            units::Voltage left_voltage = units::Voltage::from_volts(drive_output - turn_output);
            units::Voltage right_voltage = units::Voltage::from_volts(drive_output + turn_output);

            // --- Telemetry ---
            auto corner_pose = get_pose_alliance_corner();
            update_lateral_telemetry(dist_error, drive_output,
                                     units::Length::from_inches(0), dist_error, dt);
            update_angular_telemetry(heading_error_rad, turn_output,
                                     target_theta, theta, dt);
            update_pose_telemetry(corner_pose);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, drive_settle_count,
                                        telemetry::SettlementReason::NOT_SETTLED, start_time);

            move_left_motors(left_voltage);
            move_right_motors(right_voltage);
            telemetry::g_telemetry.swap();

            pros::delay(10);
        }

        // --- Timeout ---
        bool timed_out = (pros::millis() - start_time) >= config.timeout.to_milliseconds();
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            auto &data = telemetry::g_telemetry.get_write_buffer();
            if (timed_out && !data.is_settled)
            {
                data.settlement_reason = telemetry::SettlementReason::TIMEOUT;
                data.time_to_settle = units::Time::from_milliseconds(
                    pros::millis() - start_time);
            }
        }

        left_motors->brake();
        right_motors->brake();
    }

}