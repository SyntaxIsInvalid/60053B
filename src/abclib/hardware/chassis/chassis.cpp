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
#include "abclib/estimation/ekf_odometry_estimator.hpp"
#include "abclib/estimation/estimator_factory.hpp"
#include "abclib/configs/robot_selection.hpp"
#ifdef ROBOT_TEST_DRIVE
#include "abclib/configs/test_robot.hpp"
#elif defined(ROBOT_COMPETITION)
#include "abclib/configs/competition_robot.hpp"
#else
#error "No robot configuration selected!"
#endif

using namespace abclib;

namespace abclib::hardware
{

    Chassis::Chassis(ChassisConfig chassis_config, Sensors sensors)
        : left_motors(chassis_config.left),
          right_motors(chassis_config.right),
          track_width(chassis_config.track_width),
          wheel_diameter(chassis_config.diameter),
          imu(sensors.imu),
          lateral_pid(chassis_config.controllers.lateral_pid),
          angular_pid(chassis_config.controllers.angular_pid),
          ticks(chassis_config.left->get_ticks()),
          config_(chassis_config),
          alliance_(field::Alliance::BLUE)
    {
        settlement_config_ = chassis_config.controllers.settlement;
        // Create measurement models
        auto vertical_model = new estimation::WheelMeasurementModel(
            sensors.motor_y_encoder ? static_cast<hardware::ITrackingWheel *>(sensors.motor_y_encoder) : static_cast<hardware::ITrackingWheel *>(sensors.y_encoder));

        auto horizontal_model = (sensors.x_encoder || sensors.motor_x_encoder) ? new estimation::WheelMeasurementModel(
                                                                                     sensors.motor_x_encoder ? static_cast<hardware::ITrackingWheel *>(sensors.motor_x_encoder) : static_cast<hardware::ITrackingWheel *>(sensors.x_encoder))
                                                                               : nullptr;

        auto imu_model = new estimation::IMUMeasurementModel(sensors.imu);

        // Get offsets
        units::Length vertical_offset = sensors.motor_y_encoder ? sensors.motor_y_encoder->get_offset() : sensors.y_encoder->get_offset();
        units::Length horizontal_offset = (sensors.x_encoder || sensors.motor_x_encoder) ? (sensors.motor_x_encoder ? sensors.motor_x_encoder->get_offset() : sensors.x_encoder->get_offset()) : units::Length::from_inches(0.0);

        // Get estimator configuration from robot config
        auto estimator_config = robot_config::get_estimator_config();
        estimator_config.vertical_offset = vertical_offset;
        estimator_config.horizontal_offset = horizontal_offset;

        // Use factory to create the appropriate estimator
        estimator_ = estimation::create_estimator(
            estimator_config,
            vertical_model,
            horizontal_model,
            imu_model);
        /*
        path_follower_ = std::make_unique<trajectory::PathFollower>(
            this,
            chassis_config.controllers.ramsete);
        */
    }

    Chassis::~Chassis()
    {
        estimator_->stop();
    }

    void Chassis::drive(int throttle, int turn, double throttle_coefficient, double turn_coefficient)
    {
        double scaled_throttle = throttle * throttle_coefficient;
        double scaled_turn = turn * turn_coefficient;

        double left_power = scaled_throttle + scaled_turn;
        double right_power = scaled_throttle - scaled_turn;

        // Apply a deadband to avoid jitter
        if (fabs(scaled_throttle) + fabs(scaled_turn) < 2)
        {
            left_motors->brake();
            right_motors->brake();
        }
        else
        {
            // Convert from controller units (-127 to +127) to voltage (-12V to +12V)
            // PROS uses -127 to 127 range, we need to convert to -12V to 12V
            double left_voltage = (left_power / 127.0) * 12.0;
            double right_voltage = (right_power / 127.0) * 12.0;

            left_motors->move_voltage(units::Voltage::from_volts(left_voltage));
            right_motors->move_voltage(units::Voltage::from_volts(right_voltage));
        }
    }

    void Chassis::move_left_motors(units::Voltage voltage)
    {
        left_motors->move_voltage(voltage);
    }

    void Chassis::move_right_motors(units::Voltage voltage)
    {
        right_motors->move_voltage(voltage);
    }

    void Chassis::calibrate(CalibrationCallback progress_callback)
    {
        if (progress_callback)
            progress_callback(10, "Resetting sensors...");

        estimator_->stop();
        left_motors->reset_position();
        right_motors->reset_position();

        if (progress_callback)
            progress_callback(20, "Calibrating IMU...");
        imu->reset();

        // Wait for IMU to calibrate with progress updates
        int progress = 20;
        uint32_t start_time = pros::millis();
        while (imu->is_calibrating())
        {
            pros::delay(50);
            uint32_t elapsed = pros::millis() - start_time;
            // IMU takes ~2000ms to calibrate
            progress = 20 + (int)((elapsed / 2000.0) * 70);
            if (progress > 90)
                progress = 90;
            if (progress_callback)
                progress_callback(progress, "Calibrating IMU...");
        }

        if (progress_callback)
            progress_callback(90, "Finalizing...");
        pros::delay(100);

        estimator_->calibrate();
        estimator_->init();

        if (progress_callback)
            progress_callback(100, "Complete!");
    }

    void Chassis::reset_chassis_position()
    {
        left_motors->reset_position();
        right_motors->reset_position();
        imu->set_heading(0);
    }

    units::Angle Chassis::get_heading()
    {
        return units::Angle::from_radians(-imu->get_heading() * M_PI / 180.0);
    }

    void Chassis::move_voltage(units::Voltage left_voltage, units::Voltage right_voltage)
    {
        left_motors->move_voltage(left_voltage);
        right_motors->move_voltage(right_voltage);
    }

    void Chassis::move_velocity(units::Velocity left_velocity,
                                units::Velocity right_velocity,
                                double left_acceleration,
                                double right_acceleration)
    {
        units::Length wheel_radius = get_wheel_radius();

        units::AngularVelocity left_motor_vel =
            units::AngularVelocity::from_rad_per_sec(left_velocity.to_ips() / wheel_radius.to_inches());
        units::AngularVelocity right_motor_vel =
            units::AngularVelocity::from_rad_per_sec(right_velocity.to_ips() / wheel_radius.to_inches());

        // Convert wheel acceleration to motor acceleration
        double left_motor_accel = left_acceleration / wheel_radius.to_inches();
        double right_motor_accel = right_acceleration / wheel_radius.to_inches();

        left_motors->move_velocity_continuous(left_motor_vel, left_motor_accel);
        right_motors->move_velocity_continuous(right_motor_vel, right_motor_accel);

        // Update telemetry
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            auto &data = telemetry::g_telemetry.get_write_buffer();
            data.left_motor_target_velocity = left_motor_vel;
            data.right_motor_target_velocity = right_motor_vel;
        }
        update_motor_velocity_telemetry();
    }

    void Chassis::move_velocity(units::Velocity left_velocity,
                                units::Velocity right_velocity,
                                double left_acceleration,
                                double right_acceleration,
                                double override_kS,
                                double override_kV,
                                double override_kA)
    {
        units::Length wheel_radius = get_wheel_radius();

        units::AngularVelocity left_motor_vel =
            units::AngularVelocity::from_rad_per_sec(left_velocity.to_ips() / wheel_radius.to_inches());
        units::AngularVelocity right_motor_vel =
            units::AngularVelocity::from_rad_per_sec(right_velocity.to_ips() / wheel_radius.to_inches());

        double left_motor_accel = left_acceleration / wheel_radius.to_inches();
        double right_motor_accel = right_acceleration / wheel_radius.to_inches();

        left_motors->move_velocity_continuous(left_motor_vel, left_motor_accel, override_kS, override_kV, override_kA);
        right_motors->move_velocity_continuous(right_motor_vel, right_motor_accel, override_kS, override_kV, override_kA);

        // Update telemetry
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            auto &data = telemetry::g_telemetry.get_write_buffer();
            data.left_motor_target_velocity = left_motor_vel;
            data.right_motor_target_velocity = right_motor_vel;
        }
        update_motor_velocity_telemetry();
    }

    void Chassis::stop_motors()
    {
        // Stop any running velocity control tasks
        left_motors->stop_all_tasks();
        right_motors->stop_all_tasks();
    }

    void Chassis::move_velocity_pros(units::Velocity left_velocity,
                                     units::Velocity right_velocity)
    {
        // Convert linear wheel velocity to angular motor velocity
        units::Length wheel_radius = get_wheel_radius();

        units::AngularVelocity left_motor_vel =
            units::AngularVelocity::from_rad_per_sec(left_velocity.to_ips() / wheel_radius.to_inches());
        units::AngularVelocity right_motor_vel =
            units::AngularVelocity::from_rad_per_sec(right_velocity.to_ips() / wheel_radius.to_inches());

        left_motors->move_velocity_pros(left_motor_vel);
        right_motors->move_velocity_pros(right_motor_vel);

        // Update telemetry
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            auto &data = telemetry::g_telemetry.get_write_buffer();
            data.left_motor_target_velocity = left_motor_vel;
            data.right_motor_target_velocity = right_motor_vel;
        }
        update_motor_velocity_telemetry();
    }

    void Chassis::enable_distance_correction(bool enable)
    {
        if (estimator_)
        {
            auto *geo_estimator = dynamic_cast<estimation::GeometricOdometryEstimator *>(estimator_.get());
            if (geo_estimator)
            {
                geo_estimator->enable_distance_correction(enable);
            }
        }
    }

    void Chassis::set_pose(units::Length x, units::Length y, units::Angle heading)
    {
        // Default behavior: alliance corner frame (most common use case)
        set_pose_alliance_corner(x, y, heading);
    }

    void Chassis::set_pose_alliance_corner(units::Length x, units::Length y, units::Angle heading)
    {
        // Create pose in alliance corner frame
        estimation::Pose corner_pose;
        corner_pose.se2 = math::SE2(x.to_inches(), y.to_inches(), heading.to_radians());
        corner_pose.v = units::Velocity::from_ips(0.0);
        corner_pose.omega = units::AngularVelocity::from_rad_per_sec(0.0);

        // Convert to standard frame (what estimator uses internally)
        estimation::Pose standard_pose = field::alliance_corner_to_standard(
            corner_pose,
            alliance_,
            config_.field_config);

        // Set the estimator's pose (in standard frame)
        estimator_->set_pose(standard_pose);
    }

    void Chassis::set_pose_standard(units::Length x, units::Length y, units::Angle heading)
    {
        // Create pose directly in standard frame
        estimation::Pose standard_pose;
        standard_pose.se2 = math::SE2(x.to_inches(), y.to_inches(), heading.to_radians());
        standard_pose.v = units::Velocity::from_ips(0.0);
        standard_pose.omega = units::AngularVelocity::from_rad_per_sec(0.0);

        // No conversion needed - already in standard frame
        estimator_->set_pose(standard_pose);
    }

    estimation::Pose Chassis::get_pose() const
    {
        // Wrapper - defaults to alliance corner frame (what users expect)
        return get_pose_alliance_corner();
    }

    estimation::Pose Chassis::get_pose_alliance_corner() const
    {
        // Get the internal pose (standard frame)
        estimation::Pose standard_pose = estimator_->get_pose();

        // Convert to alliance corner frame
        estimation::Pose corner_pose = field::standard_to_alliance_corner(
            standard_pose,
            alliance_,
            config_.field_config);

        return corner_pose;
    }

    estimation::Pose Chassis::get_pose_standard() const
    {
        // Return directly from estimator (already in standard frame)
        return estimator_->get_pose();
    }

    void Chassis::configure_distance_correction(
        units::Length sensor_offset_forward,
        units::Length sensor_offset_lateral,
        double blend_factor)
    {
        if (estimator_)
        {
            auto *geo_estimator = dynamic_cast<estimation::GeometricOdometryEstimator *>(estimator_.get());
            if (geo_estimator)
            {
                geo_estimator->set_distance_sensor_offset(sensor_offset_forward, sensor_offset_lateral);
                geo_estimator->set_distance_blend_factor(blend_factor);
            }
        }
    }

    bool Chassis::is_distance_correction_enabled() const
    {
        if (estimator_)
        {
            auto *geo_estimator = dynamic_cast<estimation::GeometricOdometryEstimator *>(estimator_.get());
            if (geo_estimator)
            {
                return geo_estimator->is_distance_correction_enabled();
            }
        }
        return false;
    }

}