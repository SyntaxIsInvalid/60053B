#pragma once
#include "api.h"
#include "abclib/units/units.hpp"

namespace abclib
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
        units::Distance lateral_error = units::Distance::from_inches(0);
        units::Voltage lateral_output = units::Voltage::from_volts(0);
        units::Distance lateral_target = units::Distance::from_inches(0);
        units::Distance lateral_actual = units::Distance::from_inches(0);
        double lateral_p_term = 0;
        double lateral_i_term = 0;
        double lateral_d_term = 0;
        units::Distance max_lateral_error = units::Distance::from_inches(0);
        units::Distance cumulative_lateral_error = units::Distance::from_inches(0);

        // Angular control (turning)
        units::Radians angular_error = units::Radians(0); // Changed
        units::Voltage angular_output = units::Voltage::from_volts(0);
        units::Radians angular_target = units::Radians(0); // Changed
        units::Radians angular_actual = units::Radians(0); // Changed
        double angular_p_term = 0;
        double angular_i_term = 0;
        double angular_d_term = 0;
        units::Radians max_angular_error = units::Radians(0);        // Changed
        units::Radians cumulative_angular_error = units::Radians(0); // Changed

        // Pose (from odometry)
        units::BodyPose pose = units::BodyPose();
        units::BodyLinearVelocity pose_v = units::BodyLinearVelocity(0);
        units::BodyAngularVelocity pose_omega = units::BodyAngularVelocity(0);

        // Raw velocities (before filtering)
        units::BodyLinearVelocity pose_v_raw = units::BodyLinearVelocity(0);
        units::BodyAngularVelocity pose_omega_raw = units::BodyAngularVelocity(0);

        // Settlement tracking
        bool is_settled = false;
        int settle_count = 0;
        SettlementReason settlement_reason = SettlementReason::NOT_SETTLED;
        units::Time time_to_settle = units::Time::from_seconds(0); // Changed

        // Motor voltages
        units::Voltage left_motor_voltage = units::Voltage::from_volts(0);
        units::Voltage right_motor_voltage = units::Voltage::from_volts(0);

        // Path tracking errors
        units::Distance cross_track_error = units::Distance::from_inches(0);
        units::Distance along_track_error = units::Distance::from_inches(0);
        units::Distance max_cross_track_error = units::Distance::from_inches(0);
        units::Distance cumulative_xte = units::Distance::from_inches(0);
        units::Distance max_along_track_error = units::Distance::from_inches(0);
        units::Distance cumulative_ate = units::Distance::from_inches(0);

        // Final pose error
        units::Distance final_pose_error_x = units::Distance::from_inches(0);
        units::Distance final_pose_error_y = units::Distance::from_inches(0);
        units::Radians final_pose_error_theta = units::Radians(0); // Changed

        // Path follower status
        PathFollowerStatus path_status = PathFollowerStatus::IDLE;
        units::Time trajectory_time = units::Time::from_seconds(0);
        double trajectory_progress = 0.0;
        units::Time trajectory_total_time = units::Time::from_seconds(0);
        units::BodyLinearVelocity reference_velocity = units::BodyLinearVelocity(0);
        units::Distance reference_arc_position = units::Distance::from_inches(0);

        // Turn-in-place specific tracking (add near path follower section)
        units::BodyAngularVelocity omega_reference = units::BodyAngularVelocity(0);
        units::BodyAngularVelocity omega_error = units::BodyAngularVelocity(0);
        double omega_pid_output = 0.0; // raw PID output before adding to reference
        units::BodyAngularVelocity omega_commanded = units::BodyAngularVelocity(0);

        // Wheel velocity commands (what we're actually sending to motors)
        units::WheelLinearVelocity left_wheel_cmd = units::WheelLinearVelocity(0);
        units::WheelLinearVelocity right_wheel_cmd = units::WheelLinearVelocity(0);

        // Individual wheel velocities
        units::WheelAngularVelocity left_wheel_velocity = units::WheelAngularVelocity(0);
        units::WheelAngularVelocity right_wheel_velocity = units::WheelAngularVelocity(0);

        // Loop timing metrics
        units::Time loop_time = units::Time::from_seconds(0);          // Current loop duration
        units::Time max_loop_time = units::Time::from_seconds(0);      // Peak loop duration
        units::Time min_loop_time = units::Time::from_seconds(999);    // Minimum loop duration (for baseline)
        units::Time avg_loop_time = units::Time::from_seconds(0);      // Rolling average loop time
        uint32_t timing_violations = 0;                                // Count of loops exceeding target
        uint32_t total_loop_count = 0;                                 // Total loops executed
        units::Time target_loop_time = units::Time::from_millis(10.0); // Expected loop time (10ms for 100Hz)

        // Battery monitoring
        units::Voltage battery_voltage = units::Voltage::from_volts(0);
        double battery_capacity_percent = 0.0;
    };

    // Global telemetry instance
    inline TelemetryData telemetry;
    inline pros::Mutex telemetry_mutex;

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

}