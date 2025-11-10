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

    Chassis::Chassis(ChassisConfig chassis_config, Sensors sensors,
                     const control::PIDConstants lateral_constants,
                     const control::PIDConstants angular_constants)
        : left_motors(chassis_config.left),
          right_motors(chassis_config.right),
          track_width(chassis_config.track_width),
          wheel_diameter(chassis_config.diameter),
          imu(sensors.imu),
          lateral_pid(lateral_constants),
          angular_pid(angular_constants),
          ticks(chassis_config.left->get_ticks()),
          config_(chassis_config)
    {
        // Create measurement models
        auto vertical_model = new estimation::WheelMeasurementModel(
            sensors.motor_y_encoder ? static_cast<hardware::ITrackingWheel *>(sensors.motor_y_encoder) : static_cast<hardware::ITrackingWheel *>(sensors.y_encoder));

        auto horizontal_model = (sensors.x_encoder || sensors.motor_x_encoder) ? new estimation::WheelMeasurementModel(
                                                                                     sensors.motor_x_encoder ? static_cast<hardware::ITrackingWheel *>(sensors.motor_x_encoder) : static_cast<hardware::ITrackingWheel *>(sensors.x_encoder))
                                                                               : nullptr;

        auto imu_model = new estimation::IMUMeasurementModel(sensors.imu);

        // Get offsets
        units::Distance vertical_offset = sensors.motor_y_encoder ? sensors.motor_y_encoder->get_offset() : sensors.y_encoder->get_offset();
        units::Distance horizontal_offset = (sensors.x_encoder || sensors.motor_x_encoder) ? (sensors.motor_x_encoder ? sensors.motor_x_encoder->get_offset() : sensors.x_encoder->get_offset()) : units::Distance::from_inches(0.0);

        // Create estimator
        estimator_.reset(new estimation::GeometricOdometryEstimator(
            vertical_model, horizontal_model, imu_model,
            vertical_offset, horizontal_offset));
        path_follower_ = std::make_unique<trajectory::PathFollower>(this, chassis_config.ramsete_constants);
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
            left_motors->move_voltage(units::Voltage::from_pros_units(left_power));
            right_motors->move_voltage(units::Voltage::from_pros_units(right_power));
        }
    }

    void Chassis::move_left_motors(units::Voltage voltage) // Changed from double
    {
        left_motors->move_voltage(voltage);
    }

    void Chassis::move_right_motors(units::Voltage voltage) // Changed from double
    {
        right_motors->move_voltage(voltage);
    }

    void Chassis::calibrate()
    {
        estimator_->stop();
        left_motors->reset_position();
        right_motors->reset_position();
        imu->reset();

        // Wait for IMU to calibrate
        while (imu->is_calibrating())
        {
            pros::delay(10);
        }
        pros::delay(100); // Additional safety delay

        estimator_->calibrate(); // Reset sensors but keep pose
        estimator_->init();
    }

    void Chassis::reset_chassis_position()
    {
        left_motors->reset_position();
        right_motors->reset_position();
        imu->set_heading(0);
    }

    units::BodyHeading Chassis::get_heading() // Changed return type
    {
        return units::BodyHeading(units::Radians(-imu->get_heading() * M_PI / 180.0));
    }

    estimation::Pose Chassis::get_pose() const
    {
        return estimator_->get_pose();
    }

    void Chassis::set_pose(units::Distance x, units::Distance y, units::Radians heading)
    {
        // Create the new pose
        estimation::Pose new_pose;
        new_pose.pose = units::BodyPose(x.inches, y.inches, heading);
        new_pose.v = units::BodyLinearVelocity(0.0);
        new_pose.omega = units::BodyAngularVelocity(0.0);

        // Set the odometry pose (thread-safe with mutex)
        estimator_->set_pose(new_pose);

        // Set IMU heading to match (note the negation for IMU convention)
        imu->set_heading(-heading.to_degrees().value);
    }

    void Chassis::set_pose(units::Distance x, units::Distance y, units::Degrees heading)
    {
        set_pose(x, y, heading.to_radians());
    }

    void Chassis::move_voltage(units::Voltage left_voltage, units::Voltage right_voltage)
    {
        left_motors->move_voltage(left_voltage);
        right_motors->move_voltage(right_voltage);
    }
    void Chassis::move_velocity(units::WheelLinearVelocity left_velocity,
                                units::WheelLinearVelocity right_velocity,
                                double left_acceleration,
                                double right_acceleration)
    {
        units::Distance wheel_radius = get_wheel_radius();

        units::MotorAngularVelocity left_motor_vel =
            units::MotorAngularVelocity(left_velocity.inches_per_sec / wheel_radius.inches);
        units::MotorAngularVelocity right_motor_vel =
            units::MotorAngularVelocity(right_velocity.inches_per_sec / wheel_radius.inches);

        // Convert wheel acceleration to motor acceleration
        double left_motor_accel = left_acceleration / wheel_radius.inches;
        double right_motor_accel = right_acceleration / wheel_radius.inches;

        left_motors->move_velocity_continuous(left_motor_vel, left_motor_accel);
        right_motors->move_velocity_continuous(right_motor_vel, right_motor_accel);

        // Update telemetry
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            auto &data = telemetry.get_write_buffer();
            data.left_motor_target_velocity = left_motor_vel;
            data.right_motor_target_velocity = right_motor_vel;
        }
        update_motor_velocity_telemetry();
    }

    void Chassis::move_velocity(units::WheelLinearVelocity left_velocity,
                                units::WheelLinearVelocity right_velocity,
                                double left_acceleration,
                                double right_acceleration,
                                double override_kS,
                                double override_kV,
                                double override_kA)
    {
        units::Distance wheel_radius = get_wheel_radius();

        units::MotorAngularVelocity left_motor_vel =
            units::MotorAngularVelocity(left_velocity.inches_per_sec / wheel_radius.inches);
        units::MotorAngularVelocity right_motor_vel =
            units::MotorAngularVelocity(right_velocity.inches_per_sec / wheel_radius.inches);

        double left_motor_accel = left_acceleration / wheel_radius.inches;
        double right_motor_accel = right_acceleration / wheel_radius.inches;

        left_motors->move_velocity_continuous(left_motor_vel, left_motor_accel, override_kS, override_kV, override_kA);
        right_motors->move_velocity_continuous(right_motor_vel, right_motor_accel, override_kS, override_kV, override_kA);

        // Update telemetry
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            auto &data = telemetry.get_write_buffer();
            data.left_motor_target_velocity = left_motor_vel;
            data.right_motor_target_velocity = right_motor_vel;
        }
        update_motor_velocity_telemetry();
    }

    void Chassis::move_velocity_pros(units::WheelLinearVelocity left_velocity,
                                     units::WheelLinearVelocity right_velocity)
    {
        // Convert linear wheel velocity to angular motor velocity
        units::Distance wheel_radius = get_wheel_radius();

        units::MotorAngularVelocity left_motor_vel =
            units::MotorAngularVelocity(left_velocity.inches_per_sec / wheel_radius.inches);
        units::MotorAngularVelocity right_motor_vel =
            units::MotorAngularVelocity(right_velocity.inches_per_sec / wheel_radius.inches);

        left_motors->move_velocity_pros(left_motor_vel);
        right_motors->move_velocity_pros(right_motor_vel);

        // Update telemetry
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            auto &data = telemetry.get_write_buffer();
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

void Chassis::move_velocity_pros(units::WheelLinearVelocity left_velocity,
                                 units::WheelLinearVelocity right_velocity)
{
    // Convert linear wheel velocity to angular motor velocity
    units::Distance wheel_radius = get_wheel_radius();

    units::MotorAngularVelocity left_motor_vel =
        units::MotorAngularVelocity(left_velocity.inches_per_sec / wheel_radius.inches);
    units::MotorAngularVelocity right_motor_vel =
        units::MotorAngularVelocity(right_velocity.inches_per_sec / wheel_radius.inches);

    left_motors->move_velocity_pros(left_motor_vel);
    right_motors->move_velocity_pros(right_motor_vel);

    // Update telemetry
    {
        std::lock_guard<pros::Mutex> lock(telemetry_mutex);
        auto& data = telemetry.get_write_buffer();
        data.left_motor_target_velocity = left_motor_vel;
        data.right_motor_target_velocity = right_motor_vel;
    }
    update_motor_velocity_telemetry();
}

}