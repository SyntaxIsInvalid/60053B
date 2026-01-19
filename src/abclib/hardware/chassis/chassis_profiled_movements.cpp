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
#include "abclib/path/quintic_hermite_segment.hpp"
using namespace abclib;

namespace abclib::hardware
{
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
}
