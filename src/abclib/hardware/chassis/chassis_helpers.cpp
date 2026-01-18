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

    bool Chassis::check_angular_settlement(
        units::Angle error,
        units::AngularVelocity omega,
        int &settle_count) const
    {
        bool error_ok = std::abs(error.to_radians()) <= settlement_config_.angular_threshold.to_radians();
        bool velocity_ok = std::abs(omega.to_rad_per_sec()) <=
                           settlement_config_.angular_velocity_threshold.to_rad_per_sec();

        if (error_ok && velocity_ok)
        {
            settle_count++;
            return settle_count >= settlement_config_.settle_count_required;
        }

        settle_count = 0;
        return false;
    }

    bool Chassis::check_linear_settlement(
        units::Length error,
        units::Velocity velocity,
        int &settle_count) const
    {
        bool error_ok = std::abs(error.to_inches()) <= settlement_config_.position_threshold.to_inches();
        bool velocity_ok = std::abs(velocity.to_ips()) <=
                           settlement_config_.linear_velocity_threshold.to_ips();

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
        auto &data = telemetry::g_telemetry.get_write_buffer();
        data.max_lateral_error = units::Length::from_inches(0);
        data.max_angular_error = units::Angle::from_radians(0);
        data.cumulative_lateral_error = units::Length::from_inches(0);
        data.cumulative_angular_error = units::Angle::from_radians(0);
        telemetry::g_telemetry.swap();
    }

    void Chassis::update_lateral_telemetry(
        units::Length error,
        double output_volts,
        units::Length target,
        units::Length actual,
        double dt)
    {
        auto &data = telemetry::g_telemetry.get_write_buffer();

        data.lateral_error = error;
        data.lateral_output = units::Voltage::from_volts(output_volts);
        data.lateral_target = target;
        data.lateral_actual = actual;
        data.lateral_p_term = lateral_pid.get_p_term();
        data.lateral_i_term = lateral_pid.get_i_term();
        data.lateral_d_term = lateral_pid.get_d_term();

        // Accumulators
        data.max_lateral_error = units::Length::from_inches(
            std::max(data.max_lateral_error.to_inches(), std::abs(error.to_inches())));
        data.cumulative_lateral_error = units::Length::from_inches(
            data.cumulative_lateral_error.to_inches() + std::abs(error.to_inches()) * dt);
    }

    void Chassis::update_angular_telemetry(
        double error_rad,
        double output_volts,
        double target_rad,
        double actual_rad,
        double dt)
    {
        auto &data = telemetry::g_telemetry.get_write_buffer();

        data.angular_error = units::Angle::from_radians(error_rad);
        data.angular_output = units::Voltage::from_volts(output_volts);
        data.angular_target = units::Angle::from_radians(target_rad);
        data.angular_actual = units::Angle::from_radians(actual_rad);
        data.angular_p_term = angular_pid.get_p_term();
        data.angular_i_term = angular_pid.get_i_term();
        data.angular_d_term = angular_pid.get_d_term();

        // Accumulators
        data.max_angular_error = units::Angle::from_radians(
            std::max(data.max_angular_error.to_radians(), std::abs(error_rad)));
        data.cumulative_angular_error = units::Angle::from_radians(
            data.cumulative_angular_error.to_radians() + std::abs(error_rad) * dt);
    }

    void Chassis::update_pose_telemetry(const estimation::Pose &pose)
    {
        auto &data = telemetry::g_telemetry.get_write_buffer();
        data.pose_corner = pose;
    }

    void Chassis::update_motor_voltage_telemetry(
        units::Voltage left_voltage,
        units::Voltage right_voltage)
    {
        auto &data = telemetry::g_telemetry.get_write_buffer();
        data.left_motor_voltage = left_voltage;
        data.right_motor_voltage = right_voltage;
    }

    void Chassis::update_motor_velocity_telemetry()
    {
        auto &data = telemetry::g_telemetry.get_write_buffer();

        // Left motor
        data.left_motor_actual_velocity = left_motors->get_raw_velocity();
        data.left_motor_velocity_error_rpm =
            data.left_motor_target_velocity.to_rpm() -
            data.left_motor_actual_velocity.to_rpm();
        data.left_motor_velocity_p_term = left_motors->get_velocity_p_term();
        data.left_motor_velocity_i_term = left_motors->get_velocity_i_term();
        data.left_motor_velocity_d_term = left_motors->get_velocity_d_term();

        // Right motor
        data.right_motor_actual_velocity = right_motors->get_raw_velocity();
        data.right_motor_velocity_error_rpm =
            data.right_motor_target_velocity.to_rpm() -
            data.right_motor_actual_velocity.to_rpm();
        data.right_motor_velocity_p_term = right_motors->get_velocity_p_term();
        data.right_motor_velocity_i_term = right_motors->get_velocity_i_term();
        data.right_motor_velocity_d_term = right_motors->get_velocity_d_term();
    }

    void Chassis::update_settlement_telemetry(
        bool is_settled,
        int settle_count,
        telemetry::SettlementReason reason,
        uint32_t start_time)
    {
        auto &data = telemetry::g_telemetry.get_write_buffer();
        data.is_settled = is_settled;
        data.settle_count = settle_count;
        data.settlement_reason = reason;

        if (is_settled || reason != telemetry::SettlementReason::NOT_SETTLED)
        {
            data.time_to_settle = units::Time::from_milliseconds(pros::millis() - start_time);
        }
    }
}