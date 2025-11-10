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
    void Chassis::turn_to_heading(units::Degrees target_heading, units::Time timeout, units::Voltage angular_min, units::Voltage angular_max, bool reset_position)
    {
        std::uint32_t start_time = pros::millis();
        int settle_count = 0;
        const double dt = 0.01;

        // Convert target heading to radians for internal calculations
        units::Radians target_heading_rad = target_heading.to_radians();

        // Reset position if requested
        if (reset_position)
        {
            imu->set_heading(0);
        }

        // Reset PID controller
        angular_pid.reset();

        reset_telemetry_accumulators();

        while ((pros::millis() - start_time) < timeout.to_millis_uint())
        {
            // Get current heading in radians
            units::BodyHeading current_heading = get_heading();
            units::Radians current_heading_rad = current_heading.angle;

            estimation::Pose current_pose = get_pose();

            // Calculate angular error (in radians)
            double angular_error_rad = target_heading_rad.value - current_heading_rad.value;

            // Normalize heading error to [-PI, PI] for shortest turn direction
            angular_error_rad = math::normalize_angle(angular_error_rad);
            units::Radians angular_error(angular_error_rad);

            // Check if we've reached the target
            if (std::fabs(angular_error.to_degrees().value) <= settlement_config_.angular_threshold.to_degrees().value &&
                std::fabs(current_pose.omega.rad_per_sec) < settlement_config_.angular_velocity_threshold.rad_per_sec)
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

            // Calculate PID output
            double angular_output = angular_pid.compute(angular_error_rad, dt);

            // Apply min/max limits
            const double angular_coarse_threshold = 3 * M_PI / 180.0; // 3 degrees in radians
            double angular_abs = std::abs(angular_output);

            if (std::abs(angular_error_rad) > angular_coarse_threshold)
            {
                angular_abs = std::clamp(angular_abs, angular_min.volts, angular_max.volts);
                angular_output = (angular_error_rad >= 0) ? angular_abs : -angular_abs;
            }
            else
            {
                angular_output = std::clamp(angular_output, -angular_max.volts, angular_max.volts);
            }

            // Calculate motor voltages
            units::Voltage left_voltage = units::Voltage(-angular_output);
            units::Voltage right_voltage = units::Voltage(angular_output);

            // Update telemetry
            update_angular_telemetry(angular_error_rad, angular_output, target_heading_rad.value, current_heading_rad.value, dt);
            update_pose_telemetry(current_pose);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, settle_count, SettlementReason::NOT_SETTLED, start_time);

            // Apply turn power to motors (opposite directions for turning)
            move_left_motors(left_voltage);
            move_right_motors(right_voltage);

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

        // Stop motors when done
        left_motors->brake();
        right_motors->brake();
    }

    void Chassis::turn_relative(units::Degrees angle_delta,
                                units::Time timeout,
                                units::Voltage angular_min,
                                units::Voltage angular_max)
    {
        // Get current heading (returns BodyHeading with radians internally)
        units::BodyHeading current_heading = get_heading();

        // Convert current heading to degrees
        units::Degrees current_heading_deg = current_heading.angle.to_degrees();

        // Calculate target heading by adding the delta (both in degrees)
        units::Degrees target_heading = current_heading_deg + angle_delta;

        // The target will be normalized inside turn_to_heading when converted to radians
        // Call the absolute turning function
        turn_to_heading(target_heading, timeout, angular_min, angular_max, false);
    }

    void Chassis::drive_straight_relative(units::Distance target_distance,
                                          units::Time timeout,
                                          units::Voltage lateral_min,
                                          units::Voltage lateral_max,
                                          units::Voltage angular_min,
                                          units::Voltage angular_max,
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

        lateral_pid.reset();
        angular_pid.reset();

        reset_telemetry_accumulators();

        while ((pros::millis() - start_time) < timeout.to_millis_uint())
        {
            estimation::Pose current_pose = get_pose();

            // Calculate distance traveled using vector projection for accuracy
            double dx = current_pose.x() - initial_x;
            double dy = current_pose.y() - initial_y;
            double distance_traveled_raw = dx * std::cos(start_heading) + dy * std::sin(start_heading);
            units::Distance distance_traveled = units::Distance::from_inches(distance_traveled_raw);

            // Check if we've reached the target
            if (std::fabs((target_distance - distance_traveled).inches) <= settlement_config_.position_threshold.inches &&
                std::fabs(current_pose.v.inches_per_sec) < settlement_config_.linear_velocity_threshold.inches_per_sec)
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

            // Calculate lateral error and PID output
            units::Distance lateral_error = target_distance - distance_traveled;
            double lateral_output = lateral_pid.compute(lateral_error.inches, dt);

            // Calculate angular error (maintain original heading) and PID output
            double angular_error = start_heading - current_pose.theta();
            // Normalize heading error to [-PI, PI]
            angular_error = math::normalize_angle(angular_error);
            double angular_output = angular_pid.compute(angular_error, dt);

            // Apply min/max limits for forward/backward movement
            const double coarse_threshold = 0.4; // Switch to fine control at 0.4"
            double lateral_abs = std::abs(lateral_output);

            if (std::abs(lateral_error.inches) > coarse_threshold)
            {
                lateral_abs = std::clamp(lateral_abs, lateral_min.volts, lateral_max.volts);
                lateral_output = (lateral_error.inches >= 0) ? lateral_abs : -lateral_abs;
            }
            else
            {
                lateral_output = std::clamp(lateral_output, -lateral_max.volts, lateral_max.volts);
            }

            // Apply min/max limits for turning
            double angular_abs = std::abs(angular_output);
            angular_abs = std::clamp(angular_abs, angular_min.volts, angular_max.volts);
            angular_output = (angular_error >= 0) ? angular_abs : -angular_abs;

            // Calculate motor voltages
            units::Voltage left_voltage = units::Voltage::from_volts(lateral_output + angular_output);
            units::Voltage right_voltage = units::Voltage::from_volts(lateral_output - angular_output);

            // Update telemetry
            update_lateral_telemetry(lateral_error, lateral_output, target_distance, distance_traveled, dt);
            update_angular_telemetry(angular_error, angular_output, start_heading, current_pose.theta(), dt);
            update_pose_telemetry(current_pose);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, settle_count, SettlementReason::NOT_SETTLED, start_time);

            // Send power to motors
            move_left_motors(left_voltage);
            move_right_motors(right_voltage);

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
    void Chassis::euclidean_move_to_pose(
        units::Distance target_x,
        units::Distance target_y,
        units::Degrees target_heading,
        units::Time total_timeout,
        units::Time turn1_timeout,
        units::Time drive_timeout,
        units::Time turn2_timeout,
        units::Voltage lateral_min,
        units::Voltage lateral_max,
        units::Voltage angular_min,
        units::Voltage angular_max)
    {
        uint32_t start_time = pros::millis();
        uint32_t elapsed_time;

        // 1. Turn to face the target
        estimation::Pose current_pose = get_pose();
        double dx = target_x.inches - current_pose.x();
        double dy = target_y.inches - current_pose.y();
        double angle_to_target_rad = std::atan2(dy, dx);
        units::Degrees angle_to_target_deg = units::Radians(angle_to_target_rad).to_degrees();

        turn_to_heading(angle_to_target_deg, turn1_timeout, angular_min, angular_max);

        // Check total timeout
        elapsed_time = pros::millis() - start_time;
        if (elapsed_time >= total_timeout.to_millis_uint())
            return;

        // 2. RECALCULATE distance after turn
        current_pose = get_pose();
        dx = target_x.inches - current_pose.x();
        dy = target_y.inches - current_pose.y();
        double distance_to_target = std::sqrt(dx * dx + dy * dy);
        units::Distance distance = units::Distance::from_inches(distance_to_target);

        drive_straight_relative(distance, drive_timeout, lateral_min, lateral_max, angular_min, angular_max);

        // Check total timeout again
        elapsed_time = pros::millis() - start_time;
        if (elapsed_time >= total_timeout.to_millis_uint())
            return;

        // 3. Turn to final heading (already in degrees, no conversion needed)
        turn_to_heading(target_heading, turn2_timeout, angular_min, angular_max);
    }
    void Chassis::turn_to_heading_test(units::Degrees target_heading,
                                       units::Time timeout,
                                       units::Voltage angular_min,
                                       units::Voltage angular_max,
                                       bool reset_position)
    {
        std::uint32_t start_time = pros::millis();
        int settle_count = 0;
        const double dt = 0.01;

        // Convert target heading to radians for internal calculations
        units::Radians target_heading_rad = target_heading.to_radians();

        // Reset position if requested
        if (reset_position)
        {
            imu->set_heading(0);
        }

        // Reset PID controller
        angular_pid.reset();

        reset_telemetry_accumulators();

        while ((pros::millis() - start_time) < timeout.to_millis_uint())
        {
            // Get current heading in radians
            units::BodyHeading current_heading = get_heading();
            units::Radians current_heading_rad = current_heading.angle;

            estimation::Pose current_pose = get_pose();

            // Calculate angular error (in radians)
            double angular_error_rad = target_heading_rad.value - current_heading_rad.value;

            // Normalize heading error to [-PI, PI] for shortest turn direction
            angular_error_rad = math::normalize_angle(angular_error_rad);
            units::Radians angular_error(angular_error_rad);

            // Check if we've reached the target
            if (std::fabs(angular_error.to_degrees().value) <= settlement_config_.angular_threshold.to_degrees().value &&
                std::fabs(current_pose.omega.rad_per_sec) < settlement_config_.angular_velocity_threshold.rad_per_sec)
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

            // Calculate PID output
            double angular_output = angular_pid.compute(angular_error_rad, dt);

            if (std::fabs(angular_error.to_degrees().value) <= settlement_config_.angular_threshold.to_degrees().value)
            {
                double ff_voltage = 1.278592; // or whatever small voltage value you want
                angular_output += std::copysign(ff_voltage, angular_error_rad);
            }

            // Simple clamping without minimum voltage enforcement near target
            angular_output = std::clamp(angular_output, -angular_max.volts, angular_max.volts);

            // Calculate motor voltages
            units::Voltage left_voltage = units::Voltage(-angular_output);
            units::Voltage right_voltage = units::Voltage(angular_output);

            // Update telemetry
            update_angular_telemetry(angular_error_rad, angular_output, target_heading_rad.value, current_heading_rad.value, dt);
            update_pose_telemetry(current_pose);
            update_motor_voltage_telemetry(left_voltage, right_voltage);
            update_settlement_telemetry(false, settle_count, SettlementReason::NOT_SETTLED, start_time);

            // Apply turn power to motors (opposite directions for turning)
            move_left_motors(left_voltage);
            move_right_motors(right_voltage);

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

        // Stop motors when done
        left_motors->brake();
        right_motors->brake();
    }
}