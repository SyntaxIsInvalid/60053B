#pragma once

#include "api.h"
#include "abclib/hardware/motor_group.hpp"
#include "tracking_wheel.hpp"
#include "abclib/math/angles.hpp"
#include "motor_tracking_wheel.hpp"
#include "abclib/telemetry/telemetry.hpp"
// #include "abclib/trajectory/path_follower.hpp"
#include "abclib/units/units.hpp"
#include "abclib/estimation/geometric_odometry_estimator.hpp"
#include <functional>
#include "abclib/control/pure_pursuit.hpp"
#include "abclib/field/alliance.hpp"
#include "abclib/field/field_config.hpp"
#include "abclib/field/coordinate_transform.hpp"
#include "abclib/builder/path.hpp"
#include "abclib/control/ramsete.hpp"
namespace abclib::hardware
{

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

    struct SettlementConfig
    {
        units::Angle angular_threshold = units::Angle::from_degrees(1);     // Changed from Radians
        units::Length position_threshold = units::Length::from_inches(0.5); // Changed from Distance
        units::AngularVelocity angular_velocity_threshold = units::AngularVelocity::from_rad_per_sec(0.1);
        units::Velocity linear_velocity_threshold = units::Velocity::from_ips(0.15);
        int settle_count_required = 3;
    };

    struct ChainConfig
    {
        units::Voltage min_voltage = units::Voltage::from_volts(0);
        units::Length exit_distance = units::Length::from_inches(0);
        units::Angle exit_angle = units::Angle::from_degrees(0);
    };

    struct BoomerangConfig
    {
        double lead_distance = 0.6;      // 0-1, how far behind target the carrot starts
        double lateral_correction = 2.5; // aggressiveness of lateral error correction

        units::Voltage drive_max = units::Voltage::from_volts(12);
        units::Voltage drive_min = units::Voltage::from_volts(0);
        units::Voltage turn_max = units::Voltage::from_volts(6);
        units::Time timeout = units::Time::from_seconds(5);

        enum class Direction
        {
            FORWARD,
            REVERSE,
            FLEXIBLE
        };
        Direction direction = Direction::FORWARD;
    };

    struct ControllerConfig
    {
        control::PIDConstants lateral_pid;
        control::PIDConstants angular_pid;

        std::optional<filters::FilterConfig> lateral_filter;
        std::optional<filters::FilterConfig> angular_filter;

        control::PIDConstants profiled_turn_pid;
        control::PIDConstants profiled_lateral_pid;

        double turn_in_place_kS;
        double turn_in_place_kV;
        double turn_in_place_kA;
        double lateral_kS;
        double lateral_kV;
        double lateral_kA;

        SettlementConfig settlement;
        control::RamseteConstants ramsete;
    };

    struct ChassisConfig
    {
        // Hardware (required - no defaults)
        hardware::AdvancedMotorGroup *left;
        hardware::AdvancedMotorGroup *right;
        units::Length diameter;
        units::Length track_width;

        // Field configuration (optional - has default)
        field::FieldConfig field_config = field::FieldConfig::standard_vex();

        // All controller configuration (optional - has defaults)
        ControllerConfig controllers = ControllerConfig{};

        // Motor control mode (optional - has default)
        bool use_pros_controller = false;
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
        field::Alliance alliance_;
        double ticks;
        // std::unique_ptr<trajectory::PathFollower> path_follower_;

        SettlementConfig settlement_config_;
        bool has_reached_xy_goal_ = false;
        bool check_angular_settlement(units::Angle error,           // Changed from Radians
                                      units::AngularVelocity omega, // Changed from BodyAngularVelocity
                                      int &settle_count) const;

        bool check_linear_settlement(units::Length error,      // Changed from Distance
                                     units::Velocity velocity, // Changed from BodyLinearVelocity
                                     int &settle_count) const;

        double find_closest_arc_length_on_path(
            const path::Path &path,
            const estimation::Pose &robot_pose) const;

    public:
        using CalibrationCallback = std::function<void(int, const char *)>;

        Chassis(ChassisConfig chassis_config, Sensors sensors);

        ~Chassis();
        void drive(int throttle, int turn, double throttle_coefficient, double turn_coefficient);
        units::Length get_wheel_radius() const { return wheel_diameter / 2.0; } // Changed return type
        void move_left_motors(units::Voltage voltage);
        void move_right_motors(units::Voltage voltage);
        void set_alliance(field::Alliance alliance) { alliance_ = alliance; }
        field::Alliance get_alliance() const { return alliance_; }
        estimation::Pose get_pose_alliance_corner() const;
        estimation::Pose get_pose_standard() const;
        void calibrate(CalibrationCallback progress_callback = nullptr);
        void reset_chassis_position();
        units::Angle get_heading(); // Changed return type from BodyHeading

        estimation::Pose get_pose() const;
        const ChassisConfig &get_config() const { return config_; }

        void drive_straight_relative(units::Length target_distance,
                                     units::Time timeout = units::Time::from_seconds(5),
                                     units::Voltage lateral_min = units::Voltage::from_volts(0),
                                     units::Voltage lateral_max = units::Voltage::from_volts(12),
                                     units::Voltage angular_min = units::Voltage::from_volts(0),
                                     units::Voltage angular_max = units::Voltage::from_volts(0),
                                     bool reset_position = false);

        void turn_to_heading(units::Angle target_heading,
                             units::Time timeout = units::Time::from_seconds(3),
                             units::Voltage angular_max = units::Voltage::from_volts(6),
                             bool reset_position = false);

        void turn_relative(units::Angle angle_delta,
                           units::Time timeout = units::Time::from_seconds(3),
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

        void boomerang_move_to_pose(
            units::Length target_x,
            units::Length target_y,
            units::Angle target_heading,
            const BoomerangConfig &config = BoomerangConfig{});

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
        void update_ramsete_telemetry(
            const trajectory::TrajectoryState &ref_state,
            const control::RamseteOutput &output,
            const estimation::Pose &current_pose,
            double curvature,
            double k_gain);

        /*
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
        */
        void move_velocity_pros(units::Velocity left_velocity,   // Changed from WheelLinearVelocity
                                units::Velocity right_velocity); // Changed from WheelLinearVelocity

        void reset_pure_pursuit_state()
        {
            has_reached_xy_goal_ = false;
        }

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
            config_.controllers.profiled_turn_pid = constants;
        }

        void set_ramsete_constants(const control::RamseteConstants &constants)
        {
            config_.controllers.ramsete = constants;
        }

        void set_turn_in_place_feedforward(double kS, double kV, double kA)
        {
            config_.controllers.turn_in_place_kS = kS;
            config_.controllers.turn_in_place_kV = kV;
            config_.controllers.turn_in_place_kA = kA;
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

        void follow_path_ramsete(
            const path::Path &path,
            units::Time timeout = units::Time::from_seconds(15));

        void quintic_ramsete(
            units::Length target_x,
            units::Length target_y,
            units::Angle target_heading,
            units::Velocity max_velocity,
            units::Acceleration max_acceleration,
            units::Time timeout = units::Time::from_seconds(5));

        void follow_path_pure_pursuit(
            const path::Path &path,
            const control::PurePursuitConfig &config,
            units::Time timeout = units::Time::from_seconds(15));

        void quintic_pure_pursuit(
            units::Length target_x,
            units::Length target_y,
            units::Angle target_heading,
            const control::PurePursuitConfig &config,
            units::Time timeout = units::Time::from_seconds(5));

        void move_to_pose_profiled(
            units::Length target_x,
            units::Length target_y,
            units::Angle target_heading,
            units::Velocity max_velocity,
            units::Acceleration max_acceleration,
            units::Time timeout);

        void set_pose_alliance_corner(units::Length x, units::Length y, units::Angle heading);
        void set_pose_standard(units::Length x, units::Length y, units::Angle heading);
        void enable_distance_correction(bool enable);

        bool set_sensor_blending_enabled(bool enabled);
        bool is_sensor_blending_enabled() const;
        bool set_blend_config(const estimation::BlendingConfig &config);
    };
}