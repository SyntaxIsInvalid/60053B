#include "abclib/hardware/chassis.hpp"
#include "api.h"
#include <mutex>
#include "abclib/path/straight_segment.hpp"
#include "abclib/path/turn_in_place_segment.hpp"
#include <fstream>
#include "abclib/kinematics/differential_drive.hpp"
#include "abclib/estimation/wheel_measurement_models.hpp"
#include "abclib/estimation/imu_measurement_model.hpp"
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
          config_(chassis_config),
          path_follower_(std::make_unique<trajectory::PathFollower>(this, chassis_config.ramsete_constants))
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

    void Chassis::turn_to_heading(units::Degrees target_heading, units::Time timeout, units::Voltage angular_min, units::Voltage angular_max, bool reset_position)
    {
        std::uint32_t start_time = pros::millis();
        const units::Degrees threshold(1.0); // 1 degree threshold
        const int settle_count_required = 5; // 50 ms
        int settle_count = 0;
        const double dt = 0.01;

        // Convert target heading to radians for internal calculations
        units::Radians target_heading_rad = target_heading.to_radians();

        // Reset position if requested
        if (reset_position)
        {
            imu->set_heading(0);
        }

        // Reset PID controller
        angular_pid.reset();

        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            telemetry.max_angular_error = units::Radians(0);
            telemetry.cumulative_angular_error = units::Radians(0);
        }

        while ((pros::millis() - start_time) < timeout.to_millis_uint())

        {
            // Get current heading in radians
            units::BodyHeading current_heading = get_heading();
            units::Radians current_heading_rad = current_heading.angle;

            // Calculate angular error (in radians)
            double angular_error_rad = target_heading_rad.value - current_heading_rad.value;

            // Normalize heading error to [-PI, PI] for shortest turn direction
            angular_error_rad = math::normalize_angle(angular_error_rad);
            units::Radians angular_error(angular_error_rad);

            // Check if we've reached the target
            if (std::fabs(angular_error.to_degrees().value) <= threshold.value)
            {
                settle_count++;
                if (settle_count >= settle_count_required)
                {
                    {
                        std::lock_guard<pros::Mutex> lock(telemetry_mutex);
                        telemetry.is_settled = true;
                        telemetry.settle_count = settle_count;
                        telemetry.settlement_reason = SettlementReason::WITHIN_THRESHOLD;
                        telemetry.time_to_settle = units::Time::from_millis(pros::millis() - start_time);
                    }
                    left_motors->brake();
                    right_motors->brake();
                    break;
                }
            }
            else
            {
                settle_count = 0;
            }

            // Calculate PID output
            double angular_output = angular_pid.compute(angular_error_rad, dt);

            // Apply min/max limits
            /*
            double angular_abs = std::abs(angular_output);
            angular_abs = std::clamp(angular_abs, angular_min, angular_max);
            angular_output = (angular_error >= 0) ? angular_abs : -angular_abs;
            */
            const double angular_coarse_threshold = 3 * M_PI / 180.0; // 10 degrees in radians
            double angular_abs = std::abs(angular_output);

            if (std::abs(angular_error_rad) > angular_coarse_threshold)
            {
                angular_abs = std::clamp(angular_abs, angular_min.volts, angular_max.volts);
                angular_output = (angular_error_rad >= 0) ? angular_abs : -angular_abs;
            }
            else
            {
                angular_output = std::clamp(angular_output, -angular_max.volts, angular_max.volts);
            }

            // Calculate motor voltages
            units::Voltage left_voltage = units::Voltage(-angular_output);
            units::Voltage right_voltage = units::Voltage(angular_output);

            // Get current pose for telemetry
            estimation::Pose current_pose = get_pose();

            // Update telemetry
            {
                std::lock_guard<pros::Mutex> lock(telemetry_mutex);

                // Angular control
                telemetry.angular_error = angular_error;
                telemetry.angular_output = units::Voltage(angular_output);
                telemetry.angular_target = target_heading_rad;
                telemetry.angular_actual = current_heading_rad;
                telemetry.angular_p_term = angular_pid.get_p_term();
                telemetry.angular_i_term = angular_pid.get_i_term();
                telemetry.angular_d_term = angular_pid.get_d_term();

                // Pose (from odometry)
                telemetry.pose = current_pose.pose;
                telemetry.pose_v = current_pose.v;
                telemetry.pose_omega = current_pose.omega;

                // Settlement tracking
                telemetry.is_settled = false;
                telemetry.settle_count = settle_count;
                telemetry.settlement_reason = SettlementReason::NOT_SETTLED;

                telemetry.max_angular_error = units::Radians(
                    std::max(telemetry.max_angular_error.value, std::abs(angular_error_rad)));
                telemetry.cumulative_angular_error = units::Radians(
                    telemetry.cumulative_angular_error.value + std::abs(angular_error_rad) * dt);

                // Motor voltages
                telemetry.left_motor_voltage = left_voltage;
                telemetry.right_motor_voltage = right_voltage;
            }

            // Apply turn power to motors (opposite directions for turning)
            move_left_motors(left_voltage);
            move_right_motors(right_voltage);

            pros::delay(10);
        }

        // Check if we timed out
        bool timed_out = (pros::millis() - start_time) >= timeout.to_millis_uint();
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            if (timed_out && !telemetry.is_settled)
            {
                telemetry.settlement_reason = SettlementReason::TIMEOUT;
                telemetry.time_to_settle = units::Time::from_millis(pros::millis() - start_time);
            }
        }

        // Stop motors when done
        left_motors->brake();
        right_motors->brake();
    }

    void Chassis::turn_relative(units::Degrees angle_delta,
                                units::Time timeout,
                                units::Voltage angular_min,
                                units::Voltage angular_max)
    {
        // Get current heading (returns BodyHeading with radians internally)
        units::BodyHeading current_heading = get_heading();

        // Convert current heading to degrees
        units::Degrees current_heading_deg = current_heading.angle.to_degrees();

        // Calculate target heading by adding the delta (both in degrees)
        units::Degrees target_heading = current_heading_deg + angle_delta;

        // The target will be normalized inside turn_to_heading when converted to radians
        // Call the absolute turning function
        turn_to_heading(target_heading, timeout, angular_min, angular_max, false);
    }

    void Chassis::drive_straight_relative(units::Distance target_distance,
                                          units::Time timeout,
                                          units::Voltage lateral_min,
                                          units::Voltage lateral_max,
                                          units::Voltage angular_min,
                                          units::Voltage angular_max,
                                          bool reset_position)
    {
        std::uint32_t start_time = pros::millis();
        const units::Distance threshold = units::Distance::from_inches(0.5);
        const double dt = 0.01; // 100Hz

        if (reset_position)
        {
            reset_chassis_position();
        }

        estimation::Pose start_pose = get_pose();
        double start_heading = start_pose.theta();
        double initial_x = start_pose.x();
        double initial_y = start_pose.y();
        const int settle_count_required = 5; // Must be settled for 5 consecutive loops (50ms)
        int settle_count = 0;

        lateral_pid.reset();
        angular_pid.reset();

        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            telemetry.max_lateral_error = units::Distance::from_inches(0);        // TYPED
            telemetry.max_angular_error = units::Radians(0);                      // TYPED
            telemetry.cumulative_lateral_error = units::Distance::from_inches(0); // TYPED
            telemetry.cumulative_angular_error = units::Radians(0);               // TYPED
        }

        while ((pros::millis() - start_time) < timeout.to_millis_uint())
        {
            estimation::Pose current_pose = get_pose();

            // Calculate distance traveled using vector projection for accuracy
            double dx = current_pose.x() - initial_x;
            double dy = current_pose.y() - initial_y;
            double distance_traveled_raw = dx * std::cos(start_heading) + dy * std::sin(start_heading);
            units::Distance distance_traveled = units::Distance::from_inches(distance_traveled_raw);

            // Check if we've reached the target
            if (std::fabs((target_distance - distance_traveled).inches) <= threshold.inches &&
                std::fabs(current_pose.v.inches_per_sec) < 0.15) // must be nearly stopped
            {
                settle_count++;
                if (settle_count >= settle_count_required)
                {
                    // Update telemetry for successful settlement
                    {
                        std::lock_guard<pros::Mutex> lock(telemetry_mutex);
                        telemetry.is_settled = true;
                        telemetry.settle_count = settle_count;
                        telemetry.settlement_reason = SettlementReason::WITHIN_THRESHOLD;
                        telemetry.time_to_settle = units::Time::from_millis(pros::millis() - start_time); // TYPED
                    }
                    left_motors->brake();
                    right_motors->brake();
                    break;
                }
            }
            else
            {
                settle_count = 0;
            }

            // Calculate lateral error and PID output
            units::Distance lateral_error = target_distance - distance_traveled;
            double lateral_output = lateral_pid.compute(lateral_error.inches, dt);

            // Calculate angular error (maintain original heading) and PID output
            double angular_error = start_heading - current_pose.theta();
            // Normalize heading error to [-PI, PI]
            angular_error = math::normalize_angle(angular_error);
            double angular_output = angular_pid.compute(angular_error, dt);

            // Apply min/max limits for forward/backward movement
            /*
            double lateral_abs = std::abs(lateral_output);
            lateral_abs = std::clamp(lateral_abs, lateral_min, lateral_max);
            lateral_output = (lateral_error >= 0) ? lateral_abs : -lateral_abs;
            */
            const double coarse_threshold = 0.4; // Switch to fine control at 3"
            double lateral_abs = std::abs(lateral_output);

            if (std::abs(lateral_error.inches) > coarse_threshold) // ADD .inches
            {
                lateral_abs = std::clamp(lateral_abs, lateral_min.volts, lateral_max.volts); // ADD .volts
                lateral_output = (lateral_error.inches >= 0) ? lateral_abs : -lateral_abs;   // ADD .inches
            }
            else
            {
                lateral_output = std::clamp(lateral_output, -lateral_max.volts, lateral_max.volts); // ADD .volts
            }

            // Apply min/max limits for turning
            double angular_abs = std::abs(angular_output);
            angular_abs = std::clamp(angular_abs, angular_min.volts, angular_max.volts); // ADD .volts
            angular_output = (angular_error >= 0) ? angular_abs : -angular_abs;

            // Calculate motor voltages
            units::Voltage left_voltage = units::Voltage::from_volts(lateral_output + angular_output);
            units::Voltage right_voltage = units::Voltage::from_volts(lateral_output - angular_output);

            // Update telemetry
            {
                std::lock_guard<pros::Mutex> lock(telemetry_mutex);

                telemetry.lateral_error = lateral_error;                               // Already typed
                telemetry.lateral_output = units::Voltage::from_volts(lateral_output); // WRAP
                telemetry.lateral_target = target_distance;                            // Already typed
                telemetry.lateral_actual = distance_traveled;                          // Already typed
                telemetry.lateral_p_term = lateral_pid.get_p_term();
                telemetry.lateral_i_term = lateral_pid.get_i_term();
                telemetry.lateral_d_term = lateral_pid.get_d_term();

                telemetry.angular_error = units::Radians(angular_error);               // WRAP
                telemetry.angular_output = units::Voltage::from_volts(angular_output); // WRAP
                telemetry.angular_target = units::Radians(start_heading);              // WRAP
                telemetry.angular_actual = units::Radians(current_pose.theta());       // WRAP with ()
                telemetry.angular_p_term = angular_pid.get_p_term();
                telemetry.angular_i_term = angular_pid.get_i_term();
                telemetry.angular_d_term = angular_pid.get_d_term();

                telemetry.max_lateral_error = units::Distance::from_inches(
                    std::max(telemetry.max_lateral_error.inches, std::abs(lateral_error.inches)));
                telemetry.max_angular_error = units::Radians(
                    std::max(telemetry.max_angular_error.value, std::abs(angular_error)));
                telemetry.cumulative_lateral_error = units::Distance::from_inches(
                    telemetry.cumulative_lateral_error.inches + std::abs(lateral_error.inches) * dt);
                telemetry.cumulative_angular_error = units::Radians(
                    telemetry.cumulative_angular_error.value + std::abs(angular_error) * dt);

                // Pose - REMOVE the individual fields, use the BodyPose directly
                telemetry.pose = current_pose.pose;        // Assign BodyPose directly
                telemetry.pose_v = current_pose.v;         // Already typed
                telemetry.pose_omega = current_pose.omega; // Already typed

                telemetry.is_settled = false;
                telemetry.settle_count = settle_count;
                telemetry.settlement_reason = SettlementReason::NOT_SETTLED;

                telemetry.left_motor_voltage = left_voltage;   // Already typed
                telemetry.right_motor_voltage = right_voltage; // Already typed
            }

            // Send power to motors
            move_left_motors(left_voltage);
            move_right_motors(right_voltage);

            pros::delay(10);
        }

        // Check if we timed out
        bool timed_out = (pros::millis() - start_time) >= timeout.to_millis_uint(); // ADD .to_millis_uint()
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);
            if (timed_out && !telemetry.is_settled)
            {
                telemetry.settlement_reason = SettlementReason::TIMEOUT;
                telemetry.time_to_settle = units::Time::from_millis(pros::millis() - start_time); // TYPED
            }
        }

        left_motors->brake();
        right_motors->brake();
    }
    void Chassis::euclidean_move_to_pose(
        units::Distance target_x,
        units::Distance target_y,
        units::Degrees target_heading,
        units::Time total_timeout,
        units::Time turn1_timeout,
        units::Time drive_timeout,
        units::Time turn2_timeout,
        units::Voltage lateral_min,
        units::Voltage lateral_max,
        units::Voltage angular_min,
        units::Voltage angular_max)
    {
        uint32_t start_time = pros::millis();
        uint32_t elapsed_time;

        // 1. Turn to face the target
        estimation::Pose current_pose = get_pose();
        double dx = target_x.inches - current_pose.x();
        double dy = target_y.inches - current_pose.y();
        double angle_to_target_rad = std::atan2(dy, dx);
        units::Degrees angle_to_target_deg = units::Radians(angle_to_target_rad).to_degrees();

        turn_to_heading(angle_to_target_deg, turn1_timeout, angular_min, angular_max);

        // Check total timeout
        elapsed_time = pros::millis() - start_time;
        if (elapsed_time >= total_timeout.to_millis_uint())
            return;

        // 2. RECALCULATE distance after turn
        current_pose = get_pose();
        dx = target_x.inches - current_pose.x();
        dy = target_y.inches - current_pose.y();
        double distance_to_target = std::sqrt(dx * dx + dy * dy);
        units::Distance distance = units::Distance::from_inches(distance_to_target);

        drive_straight_relative(distance, drive_timeout, lateral_min, lateral_max, angular_min, angular_max);

        // Check total timeout again
        elapsed_time = pros::millis() - start_time;
        if (elapsed_time >= total_timeout.to_millis_uint())
            return;

        // 3. Turn to final heading (already in degrees, no conversion needed)
        turn_to_heading(target_heading, turn2_timeout, angular_min, angular_max);
    }

    void Chassis::move_voltage(units::Voltage left_voltage, units::Voltage right_voltage)
    {
        left_motors->move_voltage(left_voltage);
        right_motors->move_voltage(right_voltage);
    }
    void Chassis::move_velocity(units::WheelLinearVelocity left_velocity,
                                units::WheelLinearVelocity right_velocity)
    {
        // Convert linear wheel velocity to angular motor velocity
        units::Distance wheel_radius = get_wheel_radius();

        units::MotorAngularVelocity left_motor_vel =
            units::MotorAngularVelocity(left_velocity.inches_per_sec / wheel_radius.inches);
        units::MotorAngularVelocity right_motor_vel =
            units::MotorAngularVelocity(right_velocity.inches_per_sec / wheel_radius.inches);

        left_motors->move_velocity_continuous(left_motor_vel);
        right_motors->move_velocity_continuous(right_motor_vel);

        // Update telemetry
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);

            // Left motor
            telemetry.left_motor_target_velocity = left_motor_vel;
            telemetry.left_motor_actual_velocity = left_motors->get_raw_velocity();
            telemetry.left_motor_velocity_error_rpm =
                units::RPM::from_rad_per_sec(left_motor_vel.rad_per_sec).value -
                units::RPM::from_rad_per_sec(telemetry.left_motor_actual_velocity.rad_per_sec).value;
            telemetry.left_motor_velocity_p_term = left_motors->get_velocity_p_term();
            telemetry.left_motor_velocity_i_term = left_motors->get_velocity_i_term();
            telemetry.left_motor_velocity_d_term = left_motors->get_velocity_d_term();

            // Right motor
            telemetry.right_motor_target_velocity = right_motor_vel;
            telemetry.right_motor_actual_velocity = right_motors->get_raw_velocity();
            telemetry.right_motor_velocity_error_rpm =
                units::RPM::from_rad_per_sec(right_motor_vel.rad_per_sec).value -
                units::RPM::from_rad_per_sec(telemetry.right_motor_actual_velocity.rad_per_sec).value;
            telemetry.right_motor_velocity_p_term = right_motors->get_velocity_p_term();
            telemetry.right_motor_velocity_i_term = right_motors->get_velocity_i_term();
            telemetry.right_motor_velocity_d_term = right_motors->get_velocity_d_term();
        }
    }

    void Chassis::move_velocity(units::WheelLinearVelocity left_velocity,
                                units::WheelLinearVelocity right_velocity,
                                double override_kS,
                                double override_kV)
    {
        units::Distance wheel_radius = get_wheel_radius();

        units::MotorAngularVelocity left_motor_vel =
            units::MotorAngularVelocity(left_velocity.inches_per_sec / wheel_radius.inches);
        units::MotorAngularVelocity right_motor_vel =
            units::MotorAngularVelocity(right_velocity.inches_per_sec / wheel_radius.inches);

        left_motors->move_velocity_continuous(left_motor_vel, override_kS, override_kV);
        right_motors->move_velocity_continuous(right_motor_vel, override_kS, override_kV);

        // Update telemetry (same as above)
        {
            std::lock_guard<pros::Mutex> lock(telemetry_mutex);

            telemetry.left_motor_target_velocity = left_motor_vel;
            telemetry.left_motor_actual_velocity = left_motors->get_raw_velocity();
            telemetry.left_motor_velocity_error_rpm =
                units::RPM::from_rad_per_sec(left_motor_vel.rad_per_sec).value -
                units::RPM::from_rad_per_sec(telemetry.left_motor_actual_velocity.rad_per_sec).value;
            telemetry.left_motor_velocity_p_term = left_motors->get_velocity_p_term();
            telemetry.left_motor_velocity_i_term = left_motors->get_velocity_i_term();
            telemetry.left_motor_velocity_d_term = left_motors->get_velocity_d_term();

            telemetry.right_motor_target_velocity = right_motor_vel;
            telemetry.right_motor_actual_velocity = right_motors->get_raw_velocity();
            telemetry.right_motor_velocity_error_rpm =
                units::RPM::from_rad_per_sec(right_motor_vel.rad_per_sec).value -
                units::RPM::from_rad_per_sec(telemetry.right_motor_actual_velocity.rad_per_sec).value;
            telemetry.right_motor_velocity_p_term = right_motors->get_velocity_p_term();
            telemetry.right_motor_velocity_i_term = right_motors->get_velocity_i_term();
            telemetry.right_motor_velocity_d_term = right_motors->get_velocity_d_term();
        }
    }

    void Chassis::move_straight_profiled(units::Distance distance,
                                         units::BodyLinearVelocity max_velocity,
                                         double max_acceleration,
                                         units::Time timeout)
    {
        auto current = get_pose();
        path::Pose start(current.x(), current.y(), current.theta());

        // Calculate end position using raw inches for trig operations
        path::Pose end(
            current.x() + distance.inches * std::cos(current.theta()),
            current.y() + distance.inches * std::sin(current.theta()),
            current.theta());

        path::StraightSegment segment(start, end);

        trajectory::FollowerConfig config;
        config.max_velocity = max_velocity; // Already typed
        config.max_acceleration = max_acceleration;
        config.timeout = timeout; // Already typed

        path_follower_->follow_segment(&segment, config);
    }

    void Chassis::turn_to_heading_profiled(
        units::Degrees target_heading,
        units::Degrees max_angular_velocity,
        units::Degrees max_angular_acceleration,
        units::Time timeout)
    {
        // 1. Get current pose
        estimation::Pose current_pose = get_pose();

        // 2. Create turn-in-place segment
        path::Pose start(current_pose.x(), current_pose.y(), current_pose.theta());
        path::TurnInPlaceSegment segment(start, target_heading.to_radians().value, track_width);

        // 3. Convert angular velocity/acceleration to linear equivalents for the trajectory
        // For turn-in-place: v_wheel = ω_robot * (track_width / 2)
        double turning_radius = track_width.inches / 2.0;
        units::BodyLinearVelocity max_velocity = units::BodyLinearVelocity(
            max_angular_velocity.to_radians().value * turning_radius);
        double max_acceleration = max_angular_acceleration.to_radians().value * turning_radius;

        // 4. Create follower configuration
        trajectory::FollowerConfig config;
        config.max_velocity = max_velocity;
        config.max_acceleration = max_acceleration;
        config.timeout = timeout;
        config.position_threshold = units::Distance::from_inches(0.5); // not used for turn-in-place
        config.velocity_threshold = units::BodyLinearVelocity(0.1);
        config.settle_count_required = 5; // 50ms at 100Hz

        // 5. Follow the segment
        path_follower_->follow_segment(&segment, config);
    }

    void Chassis::move_to_pose_profiled(
        units::Distance target_x,
        units::Distance target_y,
        units::Degrees target_heading,
        units::BodyLinearVelocity max_velocity,
        double max_acceleration,
        units::Time timeout,
        const std::optional<path::Eta3PathSegment::EtaVec> &eta,
        const std::optional<path::Eta3PathSegment::KappaVec> &kappa)
    {
        // 1. Get current pose in body frame
        estimation::Pose current_body = get_pose();

        // 2. Convert current pose: body -> math
        double start_x_math, start_y_math, start_theta_math;
        math::body_to_math_frame(
            current_body.pose,
            start_x_math, start_y_math, start_theta_math);

        // 3. Convert target pose: body -> math
        double end_x_math, end_y_math, end_theta_math;
        units::BodyPose target_body(
            target_x.inches,
            target_y.inches,
            target_heading);
        math::body_to_math_frame(
            target_body,
            end_x_math, end_y_math, end_theta_math);

        // 4. Create Eta3 segment in math frame
        path::Pose start_math(start_x_math, start_y_math, start_theta_math);
        path::Pose end_math(end_x_math, end_y_math, end_theta_math);
        path::Eta3PathSegment segment(start_math, end_math, eta, kappa);

        // 5. Create follower configuration
        trajectory::FollowerConfig config;
        config.max_velocity = max_velocity;
        config.max_acceleration = max_acceleration;
        config.timeout = timeout;
        config.position_threshold = units::Distance::from_inches(0.5);
        config.velocity_threshold = units::BodyLinearVelocity(0.5);
        config.settle_count_required = 10; // 100ms at 100Hz

        // 6. Follow the segment using existing infrastructure
        path_follower_->follow_segment(&segment, config);
    }

    void Chassis::stop_motors()
    {
        // Stop any running velocity control tasks
        left_motors->stop_all_tasks();
        right_motors->stop_all_tasks();
    }

}