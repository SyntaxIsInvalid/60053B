using namespace abclib;
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

namespace abclib::hardware
{
    bool Chassis::check_angular_settlement(
        units::Radians error,
        units::BodyAngularVelocity omega,
        int &settle_count) const
    {
        // This is the IMPLEMENTATION - the actual logic
        bool error_ok = std::abs(error.value) <= settlement_config_.angular_threshold.value;
        bool velocity_ok = std::abs(omega.rad_per_sec) <
                           settlement_config_.angular_velocity_threshold.rad_per_sec;

        if (error_ok && velocity_ok)
        {
            settle_count++;
            return settle_count >= settlement_config_.settle_count_required;
        }

        settle_count = 0;
        return false;
    }

    bool Chassis::check_linear_settlement(
        units::Distance error,
        units::BodyLinearVelocity velocity,
        int &settle_count) const
    {
        // This is the IMPLEMENTATION - the actual logic
        bool error_ok = std::abs(error.inches) <= settlement_config_.position_threshold.inches;
        bool velocity_ok = std::abs(velocity.inches_per_sec) <
                           settlement_config_.linear_velocity_threshold.inches_per_sec;

        if (error_ok && velocity_ok)
        {
            settle_count++;
            return settle_count >= settlement_config_.settle_count_required;
        }

        settle_count = 0;
        return false;
    }

    void Chassis::reset_telemetry_accumulators()
    {
        std::lock_guard<pros::Mutex> lock(telemetry_mutex);
        telemetry.max_lateral_error = units::Distance::from_inches(0);
        telemetry.max_angular_error = units::Radians(0);
        telemetry.cumulative_lateral_error = units::Distance::from_inches(0);
        telemetry.cumulative_angular_error = units::Radians(0);
    }

    void Chassis::update_lateral_telemetry(
        units::Distance error,
        double output_volts,
        units::Distance target,
        units::Distance actual,
        double dt)
    {
        std::lock_guard<pros::Mutex> lock(telemetry_mutex);

        telemetry.lateral_error = error;
        telemetry.lateral_output = units::Voltage::from_volts(output_volts);
        telemetry.lateral_target = target;
        telemetry.lateral_actual = actual;
        telemetry.lateral_p_term = lateral_pid.get_p_term();
        telemetry.lateral_i_term = lateral_pid.get_i_term();
        telemetry.lateral_d_term = lateral_pid.get_d_term();

        // Accumulators
        telemetry.max_lateral_error = units::Distance::from_inches(
            std::max(telemetry.max_lateral_error.inches, std::abs(error.inches)));
        telemetry.cumulative_lateral_error = units::Distance::from_inches(
            telemetry.cumulative_lateral_error.inches + std::abs(error.inches) * dt);
    }

    void Chassis::update_angular_telemetry(
        double error_rad,
        double output_volts,
        double target_rad,
        double actual_rad,
        double dt)
    {
        std::lock_guard<pros::Mutex> lock(telemetry_mutex);

        telemetry.angular_error = units::Radians(error_rad);
        telemetry.angular_output = units::Voltage::from_volts(output_volts);
        telemetry.angular_target = units::Radians(target_rad);
        telemetry.angular_actual = units::Radians(actual_rad);
        telemetry.angular_p_term = angular_pid.get_p_term();
        telemetry.angular_i_term = angular_pid.get_i_term();
        telemetry.angular_d_term = angular_pid.get_d_term();

        // Accumulators
        telemetry.max_angular_error = units::Radians(
            std::max(telemetry.max_angular_error.value, std::abs(error_rad)));
        telemetry.cumulative_angular_error = units::Radians(
            telemetry.cumulative_angular_error.value + std::abs(error_rad) * dt);
    }

    void Chassis::update_pose_telemetry(const estimation::Pose &pose)
    {
        std::lock_guard<pros::Mutex> lock(telemetry_mutex);
        telemetry.pose = pose.pose;
        telemetry.pose_v = pose.v;
        telemetry.pose_omega = pose.omega;
    }

    void Chassis::update_motor_voltage_telemetry(
        units::Voltage left_voltage,
        units::Voltage right_voltage)
    {
        std::lock_guard<pros::Mutex> lock(telemetry_mutex);
        telemetry.left_motor_voltage = left_voltage;
        telemetry.right_motor_voltage = right_voltage;
    }

    void Chassis::update_motor_velocity_telemetry()
    {
        std::lock_guard<pros::Mutex> lock(telemetry_mutex);

        // Left motor
        telemetry.left_motor_actual_velocity = left_motors->get_raw_velocity();
        telemetry.left_motor_velocity_error_rpm =
            units::RPM::from_rad_per_sec(telemetry.left_motor_target_velocity.rad_per_sec).value -
            units::RPM::from_rad_per_sec(telemetry.left_motor_actual_velocity.rad_per_sec).value;
        telemetry.left_motor_velocity_p_term = left_motors->get_velocity_p_term();
        telemetry.left_motor_velocity_i_term = left_motors->get_velocity_i_term();
        telemetry.left_motor_velocity_d_term = left_motors->get_velocity_d_term();

        // Right motor
        telemetry.right_motor_actual_velocity = right_motors->get_raw_velocity();
        telemetry.right_motor_velocity_error_rpm =
            units::RPM::from_rad_per_sec(telemetry.right_motor_target_velocity.rad_per_sec).value -
            units::RPM::from_rad_per_sec(telemetry.right_motor_actual_velocity.rad_per_sec).value;
        telemetry.right_motor_velocity_p_term = right_motors->get_velocity_p_term();
        telemetry.right_motor_velocity_i_term = right_motors->get_velocity_i_term();
        telemetry.right_motor_velocity_d_term = right_motors->get_velocity_d_term();
    }

    void Chassis::update_settlement_telemetry(
        bool is_settled,
        int settle_count,
        SettlementReason reason,
        uint32_t start_time)
    {
        std::lock_guard<pros::Mutex> lock(telemetry_mutex);
        telemetry.is_settled = is_settled;
        telemetry.settle_count = settle_count;
        telemetry.settlement_reason = reason;

        if (is_settled || reason != SettlementReason::NOT_SETTLED)
        {
            telemetry.time_to_settle = units::Time::from_millis(pros::millis() - start_time);
        }
    }
}