#pragma once

#include "api.h"
#include "abclib/hardware/motor_group.hpp"
#include "tracking_wheel.hpp"
#include "abclib/math/angles.hpp"
#include "motor_tracking_wheel.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/trajectory/path_follower.hpp"
#include "abclib/units/units.hpp"
#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include <functional>
#include "abclib/control/pure_pursuit.hpp"
namespace abclib::hardware
{
    struct ChassisConfig
    {
        hardware::AdvancedMotorGroup *left;
        hardware::AdvancedMotorGroup *right;
        units::Length diameter;    // Changed from Distance
        units::Length track_width; // Changed from Distance
        control::RamseteConstants ramsete_constants = {2.0, 0.7};
        bool use_pros_controller = false;
        double turn_in_place_kS = 0.0;
        double turn_in_place_kV = 0.0;
        double turn_in_place_kA = 0.0;
        control::PIDConstants profiled_turn_pid_constants = {0.0, 0.0, 0.0};
        control::PIDConstants profiled_lateral_pid_constants = {0.0, 0.0, 0.0};
        double lateral_kS = 0.0;
        double lateral_kV = 0.0;
        double lateral_kA = 0.0;
    };

    struct Sensors
    {
        pros::IMU *imu;
        hardware::TrackingWheel *y_encoder;
        hardware::TrackingWheel *x_encoder;
        hardware::MotorTrackingWheel *motor_y_encoder = nullptr;
        hardware::MotorTrackingWheel *motor_x_encoder = nullptr;

        Sensors(pros::IMU *imu_sensor,
                hardware::TrackingWheel *y_enc = nullptr,
                hardware::TrackingWheel *x_enc = nullptr)
            : imu(imu_sensor), y_encoder(y_enc), x_encoder(x_enc) {}

        Sensors(pros::IMU *imu_sensor,
                hardware::MotorTrackingWheel *motor_y_enc,
                hardware::MotorTrackingWheel *motor_x_enc = nullptr)
            : imu(imu_sensor),
              y_encoder(nullptr), x_encoder(nullptr),
              motor_y_encoder(motor_y_enc), motor_x_encoder(motor_x_enc)
        {
        }
    };

    class Chassis
    {
    private:
        AdvancedMotorGroup *left_motors;
        AdvancedMotorGroup *right_motors;
        pros::IMU *imu;

        std::uint32_t current_time;
        std::uint32_t previous_time;
        double delta_time;

        mutable pros::Mutex telemetry_mutex;

        control::PID lateral_pid;
        control::PID angular_pid;
        std::unique_ptr<estimation::IStateEstimator> estimator_;

        units::Length track_width;    // Changed from Distance
        units::Length wheel_diameter; // Changed from Distance
        ChassisConfig config_;

        double ticks;
        std::unique_ptr<trajectory::PathFollower> path_follower_;

        struct SettlementConfig
        {
            units::Angle angular_threshold = units::Angle::from_degrees(1);     // Changed from Radians
            units::Length position_threshold = units::Length::from_inches(0.5); // Changed from Distance
            units::AngularVelocity angular_velocity_threshold =
                units::AngularVelocity::from_rad_per_sec(0.1); // Changed from BodyAngularVelocity
            units::Velocity linear_velocity_threshold =
                units::Velocity::from_ips(0.15); // Changed from BodyLinearVelocity
            int settle_count_required = 3;
        };

        SettlementConfig settlement_config_;

        bool check_angular_settlement(units::Angle error,           // Changed from Radians
                                      units::AngularVelocity omega, // Changed from BodyAngularVelocity
                                      int &settle_count) const;

        bool check_linear_settlement(units::Length error,      // Changed from Distance
                                     units::Velocity velocity, // Changed from BodyLinearVelocity
                                     int &settle_count) const;

            double find_closest_arc_length_on_path(
        const path::Path& path, 
        const estimation::Pose& robot_pose) const;

    public:
        using CalibrationCallback = std::function<void(int, const char *)>;

        Chassis(ChassisConfig chassis_config, Sensors sensors,
                const control::PIDConstants lateral_constants,
                const control::PIDConstants angular_constants);
        ~Chassis();
        void drive(int throttle, int turn, double throttle_coefficient, double turn_coefficient);
        units::Length get_wheel_radius() const { return wheel_diameter / 2.0; } // Changed return type
        void move_left_motors(units::Voltage voltage);
        void move_right_motors(units::Voltage voltage);

        void calibrate(CalibrationCallback progress_callback = nullptr);
        void reset_chassis_position();
        units::Angle get_heading(); // Changed return type from BodyHeading

        estimation::Pose get_pose() const;
        const ChassisConfig &get_config() const { return config_; }

        void drive_straight_relative(units::Length target_distance, // Changed from Distance
                                     units::Time timeout = units::Time::from_seconds(5),
                                     units::Voltage lateral_min = units::Voltage::from_volts(0),
                                     units::Voltage lateral_max = units::Voltage::from_volts(12),
                                     units::Voltage angular_min = units::Voltage::from_volts(0),
                                     units::Voltage angular_max = units::Voltage::from_volts(0),
                                     bool reset_position = false);

        void turn_to_heading(units::Angle target_heading, // Changed from Degrees
                             units::Time timeout = units::Time::from_seconds(3),
                             units::Voltage angular_min = units::Voltage::from_volts(0),
                             units::Voltage angular_max = units::Voltage::from_volts(6),
                             bool reset_position = false);

        void turn_relative(units::Angle angle_delta, // Changed from Degrees
                           units::Time timeout = units::Time::from_seconds(3),
                           units::Voltage angular_min = units::Voltage::from_volts(0),
                           units::Voltage angular_max = units::Voltage::from_volts(6));

        void euclidean_move_to_pose(
            units::Length target_x,      // Changed from Distance
            units::Length target_y,      // Changed from Distance
            units::Angle target_heading, // Changed from Degrees
            units::Time total_timeout = units::Time::from_seconds(15),
            units::Time turn1_timeout = units::Time::from_seconds(5),
            units::Time drive_timeout = units::Time::from_seconds(5),
            units::Time turn2_timeout = units::Time::from_seconds(5),
            units::Voltage lateral_min = units::Voltage::from_volts(0),
            units::Voltage lateral_max = units::Voltage::from_volts(12),
            units::Voltage angular_min = units::Voltage::from_volts(0),
            units::Voltage angular_max = units::Voltage::from_volts(6));

        void move_voltage(units::Voltage left_voltage, units::Voltage right_voltage);

        void move_velocity(units::Velocity left_velocity,  // Changed from WheelLinearVelocity
                           units::Velocity right_velocity, // Changed from WheelLinearVelocity
                           double left_acceleration = 0.0,
                           double right_acceleration = 0.0);

        void move_velocity(units::Velocity left_velocity,  // Changed from WheelLinearVelocity
                           units::Velocity right_velocity, // Changed from WheelLinearVelocity
                           double left_acceleration,
                           double right_acceleration,
                           double override_kS,
                           double override_kV,
                           double override_kA);

        void set_pose(units::Length x,       // Changed from Distance
                      units::Length y,       // Changed from Distance
                      units::Angle heading); // Changed from Radians

        void turn_to_heading_profiled_pid(
            units::Angle target_heading,
            units::AngularVelocity max_angular_velocity,
            units::AngularAcceleration max_angular_acceleration,
            units::Time timeout);

        void move_straight_profiled_pid(
            units::Length target_distance,
            units::Velocity max_velocity,
            units::Acceleration max_acceleration,
            units::Time timeout = units::Time::from_seconds(5),
            bool reset_position = false);

        units::Length get_track_width() const { return track_width; } // Changed return type

        void stop_motors();

        void follow_segment(const path::IPathSegment *segment,
                            const trajectory::FollowerConfig &config)
        {
            path_follower_->follow_segment(segment, config);
        }

        void follow_path(const path::Path &path,
                         units::Time timeout = units::Time::from_seconds(15))
        {
            path_follower_->follow_path(path, timeout);
        }

        void move_velocity_pros(units::Velocity left_velocity,   // Changed from WheelLinearVelocity
                                units::Velocity right_velocity); // Changed from WheelLinearVelocity

        void move_straight_profiled(
            units::Length distance,
            units::Velocity max_velocity,
            units::Acceleration max_acceleration, // Changed from double
            units::Time timeout = units::Time::from_seconds(5),
            double heading_tolerance = 0.1 // ~5.7 degrees tolerance for IMU drift
        );

        void turn_to_heading_profiled(
            units::Angle target_heading,
            units::AngularVelocity max_angular_velocity,         // Changed from double
            units::AngularAcceleration max_angular_acceleration, // Changed from double
            units::Time timeout = units::Time::from_seconds(3));

        void turn_to_heading_test(units::Angle target_heading, // Changed from Degrees
                                  units::Time timeout,
                                  units::Voltage angular_min,
                                  units::Voltage angular_max,
                                  bool reset_position = false);

        void set_settlement_config(const SettlementConfig &config)
        {
            settlement_config_ = config;
        }

        const SettlementConfig &get_settlement_config() const
        {
            return settlement_config_;
        }

        void reset_telemetry_accumulators();

        void update_lateral_telemetry(
            units::Length error, // Changed from Distance
            double output_volts,
            units::Length target, // Changed from Distance
            units::Length actual, // Changed from Distance
            double dt);

        void update_angular_telemetry(
            double error_rad,
            double output_volts,
            double target_rad,
            double actual_rad,
            double dt);

        void update_pose_telemetry(const estimation::Pose &pose);

        void update_motor_voltage_telemetry(
            units::Voltage left_voltage,
            units::Voltage right_voltage);

        void update_motor_velocity_telemetry();

        void update_settlement_telemetry(
            bool is_settled,
            int settle_count,
            telemetry::SettlementReason reason,
            uint32_t start_time);

        void set_lateral_pid_constants(const control::PIDConstants &constants)
        {
            lateral_pid.set_constants(constants);
        }

        void set_angular_pid_constants(const control::PIDConstants &constants)
        {
            angular_pid.set_constants(constants);
        }

        void set_profiled_turn_pid_constants(const control::PIDConstants &constants)
        {
            config_.profiled_turn_pid_constants = constants;
        }

        void set_ramsete_constants(const control::RamseteConstants &constants)
        {
            config_.ramsete_constants = constants;
        }

        void set_turn_in_place_feedforward(double kS, double kV, double kA)
        {
            config_.turn_in_place_kS = kS;
            config_.turn_in_place_kV = kV;
            config_.turn_in_place_kA = kA;
        }

        void set_motor_feedforward(double kS, double kV, double kA)
        {
            auto left_config = left_motors->get_config();
            left_config.kS = kS;
            left_config.kV = kV;
            left_config.kA = kA;
            left_motors->set_config(left_config);

            auto right_config = right_motors->get_config();
            right_config.kS = kS;
            right_config.kV = kV;
            right_config.kA = kA;
            right_motors->set_config(right_config);
        }

        void follow_path_pure_pursuit(
        const path::Path& path,
        const control::PurePursuitConfig& config,
        units::Time timeout = units::Time::from_seconds(15));
        
void quintic_pure_pursuit(
    units::Length target_x,
    units::Length target_y,
    units::Angle target_heading,
    const control::PurePursuitConfig& config,
    units::Time timeout = units::Time::from_seconds(5));

        void move_to_pose_profiled(
            units::Length target_x,
            units::Length target_y,
            units::Angle target_heading,
            units::Velocity max_velocity,
            units::Acceleration max_acceleration,
            units::Time timeout);
    };
}