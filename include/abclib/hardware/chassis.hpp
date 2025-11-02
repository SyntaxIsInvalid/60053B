#pragma once

#include "api.h"
#include "abclib/hardware/motor_group.hpp"
#include "tracking_wheel.hpp"
// #include "abclib/estimation/odometry.hpp"
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

        double turn_in_place_kS = 0.0;
        double turn_in_place_kV = 0.0;
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
                           units::WheelLinearVelocity right_velocity);
        void move_velocity(units::WheelLinearVelocity left_velocity,
                           units::WheelLinearVelocity right_velocity,
                           double override_kS,
                           double override_kV);

        void move_straight_profiled(units::Distance distance,
                                    units::BodyLinearVelocity max_velocity,
                                    double max_acceleration,
                                    units::Time timeout = units::Time::from_seconds(10));

        void turn_to_heading_profiled(
            units::Degrees target_heading,
            units::Degrees max_angular_velocity,
            units::Degrees max_angular_acceleration,
            units::Time timeout = units::Time::from_seconds(5));

        void set_pose(units::Distance x,
                      units::Distance y,
                      units::Radians heading);

        void set_pose(units::Distance x,
                      units::Distance y,
                      units::Degrees heading);

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

        void move_to_pose_profiled(
            units::Distance target_x,
            units::Distance target_y,
            units::Degrees target_heading,
            units::BodyLinearVelocity max_velocity,
            double max_acceleration,
            units::Time timeout = units::Time::from_seconds(10),
            const std::optional<Eigen::Matrix<double, 6, 1>> &custom_Q = std::nullopt,
            const std::optional<Eigen::Matrix<double, 4, 1>> &custom_R = std::nullopt);
    };
}