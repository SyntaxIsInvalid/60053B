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
#include "abclib/control/profiled_pid.hpp"
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
            units::Voltage left_voltage = units::Voltage::from_volts(lateral_output + angular_output);
            units::Voltage right_voltage = units::Voltage::from_volts(lateral_output - angular_output);

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
}