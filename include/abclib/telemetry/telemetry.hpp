#pragma once
#include "api.h"
#include "abclib/units/units.hpp"
#include "abclib/estimation/pose.hpp"
#include <atomic>
#include "abclib/field/field_map.hpp"
namespace abclib::telemetry
{

    enum class SettlementReason
    {
        NOT_SETTLED,
        WITHIN_THRESHOLD, // Reached target successfully
        TIMEOUT,          // Ran out of time
        INTERRUPTED       // Task was cancelled/interrupted
    };

    enum class PathFollowerStatus
    {
        IDLE,         // Not running
        ACCELERATING, // Speed increasing
        CRUISING,     // Constant velocity
        DECELERATING, // Slowing down
        SETTLING,     // At end, checking settlement
        COMPLETE      // Successfully finished
    };

    struct TelemetryData
    {
        // Timestamp
        units::Time timestamp = units::Time::from_seconds(0);

        // Lateral control (drive straight)
        units::Length lateral_error = units::Length::from_inches(0);
        units::Voltage lateral_output = units::Voltage::from_volts(0);
        units::Length lateral_target = units::Length::from_inches(0);
        units::Length lateral_actual = units::Length::from_inches(0);
        double lateral_p_term = 0;
        double lateral_i_term = 0;
        double lateral_d_term = 0;
        units::Length max_lateral_error = units::Length::from_inches(0);
        units::Length cumulative_lateral_error = units::Length::from_inches(0);

        // Angular control (turning)
        units::Angle angular_error = units::Angle::from_radians(0);
        units::Voltage angular_output = units::Voltage::from_volts(0);
        units::Angle angular_target = units::Angle::from_radians(0);
        units::Angle angular_actual = units::Angle::from_radians(0);
        double angular_p_term = 0;
        double angular_i_term = 0;
        double angular_d_term = 0;
        units::Angle max_angular_error = units::Angle::from_radians(0);
        units::Angle cumulative_angular_error = units::Angle::from_radians(0);

        // Pose (from odometry)
        estimation::Pose pose_corner; // Alliance corner frame (public API)
        estimation::Pose pose_center; // Field center frame (internal)

        // Raw velocities (before filtering)
        units::Velocity pose_v_raw = units::Velocity::from_ips(0);
        units::AngularVelocity pose_omega_raw = units::AngularVelocity::from_rad_per_sec(0);

        // Settlement tracking
        bool is_settled = false;
        int settle_count = 0;
        SettlementReason settlement_reason = SettlementReason::NOT_SETTLED;
        units::Time time_to_settle = units::Time::from_seconds(0);

        // Motor voltages
        units::Voltage left_motor_voltage = units::Voltage::from_volts(0);
        units::Voltage right_motor_voltage = units::Voltage::from_volts(0);

        // Path tracking errors
        units::Length cross_track_error = units::Length::from_inches(0);
        units::Length along_track_error = units::Length::from_inches(0);
        units::Length max_cross_track_error = units::Length::from_inches(0);
        units::Length cumulative_xte = units::Length::from_inches(0);
        units::Length max_along_track_error = units::Length::from_inches(0);
        units::Length cumulative_ate = units::Length::from_inches(0);

        // Final pose error
        units::Length final_pose_error_x = units::Length::from_inches(0);
        units::Length final_pose_error_y = units::Length::from_inches(0);
        units::Angle final_pose_error_theta = units::Angle::from_radians(0);

        // Path follower status
        PathFollowerStatus path_status = PathFollowerStatus::IDLE;
        units::Time trajectory_time = units::Time::from_seconds(0);
        double trajectory_progress = 0.0;
        units::Time trajectory_total_time = units::Time::from_seconds(0);
        units::Velocity reference_velocity = units::Velocity::from_ips(0);
        units::Length reference_arc_position = units::Length::from_inches(0);

        // Turn-in-place specific tracking
        units::AngularVelocity omega_reference = units::AngularVelocity::from_rad_per_sec(0);
        units::AngularVelocity omega_error = units::AngularVelocity::from_rad_per_sec(0);
        double omega_pid_output = 0.0;
        units::AngularVelocity omega_commanded = units::AngularVelocity::from_rad_per_sec(0);

        // Wheel velocity commands
        units::Velocity left_wheel_cmd = units::Velocity::from_ips(0);
        units::Velocity right_wheel_cmd = units::Velocity::from_ips(0);

        // Individual wheel velocities
        units::AngularVelocity left_wheel_velocity = units::AngularVelocity::from_rad_per_sec(0);
        units::AngularVelocity right_wheel_velocity = units::AngularVelocity::from_rad_per_sec(0);

        // Loop timing metrics
        units::Time loop_time = units::Time::from_seconds(0);
        units::Time max_loop_time = units::Time::from_seconds(0);
        units::Time min_loop_time = units::Time::from_seconds(999);
        units::Time avg_loop_time = units::Time::from_seconds(0);
        uint32_t timing_violations = 0;
        uint32_t total_loop_count = 0;
        units::Time target_loop_time = units::Time::from_milliseconds(10.0);

        // Battery monitoring
        units::Voltage battery_voltage = units::Voltage::from_volts(0);
        double battery_capacity_percent = 0.0;
        bool voltage_compensation_active = false;
        double voltage_compensation_scale = 1.0;

        // Motor velocity control (left)
        units::AngularVelocity left_motor_target_velocity = units::AngularVelocity::from_rad_per_sec(0);
        units::AngularVelocity left_motor_actual_velocity = units::AngularVelocity::from_rad_per_sec(0);
        double left_motor_velocity_error_rpm = 0;
        double left_motor_velocity_p_term = 0;
        double left_motor_velocity_i_term = 0;
        double left_motor_velocity_d_term = 0;
        units::Voltage left_motor_velocity_output = units::Voltage::from_volts(0);

        // Motor velocity control (right)
        units::AngularVelocity right_motor_target_velocity = units::AngularVelocity::from_rad_per_sec(0);
        units::AngularVelocity right_motor_actual_velocity = units::AngularVelocity::from_rad_per_sec(0);
        double right_motor_velocity_error_rpm = 0;
        double right_motor_velocity_p_term = 0;
        double right_motor_velocity_i_term = 0;
        double right_motor_velocity_d_term = 0;
        units::Voltage right_motor_velocity_output = units::Voltage::from_volts(0);

        units::Length ekf_x;
        units::Length ekf_y;
        units::Angle ekf_theta;

        // EKF uncertainty (standard deviations)
        double ekf_x_std;
        double ekf_y_std;
        double ekf_theta_std;

        // Distance sensor measurements
        units::Length front_distance_measured;
        units::Length front_distance_expected;
        bool front_distance_valid;
        field::FieldMap::Wall front_wall;

        units::Length back_distance_measured;
        units::Length back_distance_expected;
        bool back_distance_valid;
        field::FieldMap::Wall back_wall;

        // EKF innovation (measurement - prediction)
        units::Length front_innovation;
        units::Length back_innovation;

        // Kalman gain (how much we trust measurements)
        double kalman_gain_x_front;
        double kalman_gain_y_front;
        double kalman_gain_theta_front;

        field::Alliance current_alliance = field::Alliance::BLUE;
    };

    // Double-buffered telemetry system
    class TelemetryBuffer
    {
    private:
        TelemetryData buffers[2];
        std::atomic<int> read_index{0};
        int write_index = 1;

    public:
        // Writer (control loop) gets direct access to write buffer
        TelemetryData &get_write_buffer()
        {
            return buffers[write_index];
        }

        // Writer swaps buffers when done updating
        void swap()
        {
            read_index.store(write_index, std::memory_order_release);
            write_index = 1 - write_index;
        }

        // Reader (display/logging) gets const access to read buffer
        const TelemetryData &get_read_buffer() const
        {
            return buffers[read_index.load(std::memory_order_acquire)];
        }
    };

    // Global telemetry instance (double-buffered)
    inline TelemetryBuffer g_telemetry;

    // Helper function to convert enum to string for display
    inline const char *settlement_reason_to_string(SettlementReason reason)
    {
        switch (reason)
        {
        case SettlementReason::NOT_SETTLED:
            return "RUNNING";
        case SettlementReason::WITHIN_THRESHOLD:
            return "SUCCESS";
        case SettlementReason::TIMEOUT:
            return "TIMEOUT";
        case SettlementReason::INTERRUPTED:
            return "CANCEL";
        default:
            return "UNKNOWN";
        }
    }

    inline const char *path_status_to_string(PathFollowerStatus status)
    {
        switch (status)
        {
        case PathFollowerStatus::IDLE:
            return "IDLE";
        case PathFollowerStatus::ACCELERATING:
            return "ACCEL";
        case PathFollowerStatus::CRUISING:
            return "CRUISE";
        case PathFollowerStatus::DECELERATING:
            return "DECEL";
        case PathFollowerStatus::SETTLING:
            return "SETTLE";
        case PathFollowerStatus::COMPLETE:
            return "DONE";
        default:
            return "UNKNOWN";
        }
    }

} // namespace abclib::telemetry