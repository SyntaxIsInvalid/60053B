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

namespace abclib::hardware
{
    struct ChassisConfig
    {
        hardware::AdvancedMotorGroup *left;
        hardware::AdvancedMotorGroup *right;
        units::Distance diameter;
        units::Distance track_width;
        control::RamseteConstants ramsete_constants = {2.0, 0.7};
        bool use_pros_controller = false;
        double turn_in_place_kS = 0.0;
        double turn_in_place_kV = 0.0;
        double turn_in_place_kA = 0.0;
        control::PIDConstants profiled_turn_pid_constants = {0.0, 0.0, 0.0};
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
        // estimation::Odometry odom;
        std::unique_ptr<estimation::IStateEstimator> estimator_;

        units::Distance track_width;
        units::Distance wheel_diameter;
        ChassisConfig config_;

        double ticks;
        units::Distance get_wheel_radius() const { return wheel_diameter / 2.0; }
        std::unique_ptr<trajectory::PathFollower> path_follower_;

        struct SettlementConfig
        {
            units::Radians angular_threshold = units::Radians(1 * M_PI / 180.0);
            units::Distance position_threshold = units::Distance::from_inches(0.5);
            units::BodyAngularVelocity angular_velocity_threshold =
                units::BodyAngularVelocity(0.1);
            units::BodyLinearVelocity linear_velocity_threshold =
                units::BodyLinearVelocity(0.15);
            int settle_count_required = 3;
        };

        SettlementConfig settlement_config_;

        bool check_angular_settlement(units::Radians error,
                                      units::BodyAngularVelocity omega,
                                      int &settle_count) const;

        bool check_linear_settlement(units::Distance error,
                                     units::BodyLinearVelocity velocity,
                                     int &settle_count) const;

    public:
        Chassis(ChassisConfig chassis_config, Sensors sensors,
                const control::PIDConstants lateral_constants,
                const control::PIDConstants angular_constants);
        ~Chassis();
        void drive(int throttle, int turn, double throttle_coefficient, double turn_coefficient);

        void move_left_motors(units::Voltage voltage);
        void move_right_motors(units::Voltage voltage);

        void calibrate();
        void reset_chassis_position();
        units::BodyHeading get_heading();

        estimation::Pose get_pose() const;
        const ChassisConfig &get_config() const { return config_; }

        void drive_straight_relative(units::Distance target_distance,
                                     units::Time timeout = units::Time::from_seconds(5),
                                     units::Voltage lateral_min = units::Voltage::from_volts(0),
                                     units::Voltage lateral_max = units::Voltage::from_volts(12),
                                     units::Voltage angular_min = units::Voltage::from_volts(0),
                                     units::Voltage angular_max = units::Voltage::from_volts(0),
                                     bool reset_position = false);

        void turn_to_heading(units::Degrees target_heading,
                             units::Time timeout = units::Time::from_seconds(3),
                             units::Voltage angular_min = units::Voltage::from_volts(0),
                             units::Voltage angular_max = units::Voltage::from_volts(6),
                             bool reset_position = false);
        void turn_relative(units::Degrees angle_delta,
                           units::Time timeout = units::Time::from_seconds(3),
                           units::Voltage angular_min = units::Voltage::from_volts(0),
                           units::Voltage angular_max = units::Voltage::from_volts(6));

        void euclidean_move_to_pose(
            units::Distance target_x,
            units::Distance target_y,
            units::Degrees target_heading,
            units::Time total_timeout = units::Time::from_seconds(15),
            units::Time turn1_timeout = units::Time::from_seconds(5),
            units::Time drive_timeout = units::Time::from_seconds(5),
            units::Time turn2_timeout = units::Time::from_seconds(5),
            units::Voltage lateral_min = units::Voltage::from_volts(0),
            units::Voltage lateral_max = units::Voltage::from_volts(12),
            units::Voltage angular_min = units::Voltage::from_volts(0),
            units::Voltage angular_max = units::Voltage::from_volts(6));

        void move_voltage(units::Voltage left_voltage, units::Voltage right_voltage);
        void move_velocity(units::WheelLinearVelocity left_velocity,
                           units::WheelLinearVelocity right_velocity,
                           double left_acceleration = 0.0,
                           double right_acceleration = 0.0);

        void move_velocity(units::WheelLinearVelocity left_velocity,
                           units::WheelLinearVelocity right_velocity,
                           double left_acceleration,
                           double right_acceleration,
                           double override_kS,
                           double override_kV,
                           double override_kA);

        void set_pose(units::Distance x,
                      units::Distance y,
                      units::Radians heading);

        void set_pose(units::Distance x,
                      units::Distance y,
                      units::Degrees heading);

        void turn_to_heading_profiled_pid(
            units::Degrees target_heading,
            double max_angular_velocity_deg_per_sec,
            double max_angular_acceleration_deg_per_sec2,
            units::Time timeout);

        units::Distance get_track_width() const { return track_width; }
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

        void move_velocity_pros(units::WheelLinearVelocity left_velocity,
                                units::WheelLinearVelocity right_velocity);

        void move_straight_profiled(
            units::Distance distance,
            units::BodyLinearVelocity max_velocity,
            double max_acceleration,
            units::Time timeout = units::Time::from_seconds(5),
            double heading_tolerance = 0.1 // ~5.7 degrees tolerance for IMU drift
        );

        void turn_to_heading_profiled(
            units::Degrees target_heading,
            double max_body_angular_velocity_deg_per_sec,
            double max_bodyangular_acceleration_deg_per_sec2,
            units::Time timeout = units::Time::from_seconds(3));

        void turn_to_heading_test(units::Degrees target_heading,
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
            units::Distance error,
            double output_volts,
            units::Distance target,
            units::Distance actual,
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
            SettlementReason reason,
            uint32_t start_time);
    };
}