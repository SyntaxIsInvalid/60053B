#include "abclib/characterization/motor_characterization.hpp"
#include "abclib/units/units.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.h"
#include <cstdio>
#include "abclib/telemetry/telemetry.hpp"
#include <mutex>
#include "abclib/hardware/chassis.hpp"

namespace abclib::characterization
{

    void measure_ks_kv(
        hardware::AdvancedMotorGroup &left,
        hardware::AdvancedMotorGroup &right,
        bool forward,
        const char *filename,
        double max_voltage,
        double voltage_step,
        int step_duration_ms)
    {
        // Check SD card availability
        if (!pros::usd::is_installed())
        {
            pros::lcd::print(0, "ERROR: No SD card!");
            return;
        }

        // Build filename with direction suffix
        char full_filename[64];
        snprintf(full_filename, sizeof(full_filename), "/usd/%s_%s.csv",
                 filename, forward ? "forward" : "backward");

        pros::lcd::print(0, "Starting motor test...");
        pros::lcd::print(1, "Direction: %s", forward ? "FWD" : "BWD");
        pros::delay(2000);

        left.reset_position();
        right.reset_position();
        pros::delay(100); // Let encoders settle

        // Open file for writing
        FILE *file = fopen(full_filename, "w");
        if (file == nullptr)
        {
            pros::lcd::print(0, "ERROR: Can't open file!");
            return;
        }

        // Write CSV header
        fprintf(file, "time_ms,voltage_volts,left_vel_rad_s,right_vel_rad_s,avg_vel_rad_s\n");

        uint32_t test_start_time = pros::millis();
        double direction_multiplier = forward ? 1.0 : -1.0;

        pros::lcd::print(0, "Testing %s...", forward ? "forward" : "backward");

        // Voltage ramp test (0V to max_voltage in real volts, not PROS units)
        for (double voltage = 0.0; voltage <= max_voltage; voltage += voltage_step)
        {
            uint32_t step_start = pros::millis();

            // Apply voltage in specified direction using typed units
            // voltage is already in real volts (0-12V range)
            units::Voltage applied_voltage = units::Voltage::from_volts(voltage * direction_multiplier);
            left.move_voltage(applied_voltage);
            right.move_voltage(applied_voltage);

            // Sample velocities during this step
            while ((pros::millis() - step_start) < static_cast<uint32_t>(step_duration_ms))
            {
                // Get velocities using typed units (returns RPM)
                units::MotorAngularVelocity left_vel = left.get_raw_velocity();
                units::MotorAngularVelocity right_vel = right.get_raw_velocity();
                double avg_vel_rad_s = (left_vel.rad_per_sec + right_vel.rad_per_sec) / 2.0;

                uint32_t timestamp = pros::millis() - test_start_time;

                // Write to SD card (all values in real units: volts and RPM)
                fprintf(file, "%lu,%.2f,%.4f,%.4f,%.4f\n",
                        timestamp, voltage * direction_multiplier,
                        left_vel.rad_per_sec, right_vel.rad_per_sec, avg_vel_rad_s);

                pros::delay(10); // Sample at 100Hz
            }

            // Update LCD periodically
            if (static_cast<int>(voltage) % 2 == 0) // Update every 2V instead of 5V
            {
                pros::lcd::print(1, "Testing %.1fV...", voltage);
            }
        }

        // Stop motors and cleanup
        left.brake();
        right.brake();
        fclose(file);

        pros::lcd::print(0, "Test complete!");
        pros::lcd::print(1, "File: %s", full_filename);
        pros::lcd::print(2, "Analyze offline for");
        pros::lcd::print(3, "kS and kV values");
    }

    void measure_ks_kv_turn(
        hardware::AdvancedMotorGroup &left,
        hardware::AdvancedMotorGroup &right,
        bool ccw_rotation, // counter-clockwise vs clockwise
        const char *filename,
        double max_voltage,
        double voltage_step,
        int step_duration_ms)
    {
        // Check SD card
        if (!pros::usd::is_installed())
        {
            pros::lcd::print(0, "ERROR: No SD card!");
            return;
        }

        // Build filename with rotation direction
        char full_filename[64];
        snprintf(full_filename, sizeof(full_filename), "/usd/%s_turn_%s.csv",
                 filename, ccw_rotation ? "ccw" : "cw");

        pros::lcd::print(0, "Starting turn test...");
        pros::lcd::print(1, "Rotation: %s", ccw_rotation ? "CCW" : "CW");
        pros::delay(2000);

        left.reset_position();
        right.reset_position();
        pros::delay(100);

        FILE *file = fopen(full_filename, "w");
        if (file == nullptr)
        {
            pros::lcd::print(0, "ERROR: Can't open file!");
            return;
        }

        // Write CSV header with additional columns
        fprintf(file, "time_ms,voltage_volts,left_vel_rad_s,right_vel_rad_s,");
        fprintf(file, "avg_wheel_vel_rad_s,angular_vel_rad_s\n");

        uint32_t test_start_time = pros::millis();

        // For CCW rotation: left backward, right forward
        // For CW rotation: left forward, right backward
        double left_multiplier = ccw_rotation ? -1.0 : 1.0;
        double right_multiplier = ccw_rotation ? 1.0 : -1.0;

        pros::lcd::print(0, "Testing turn %s...", ccw_rotation ? "CCW" : "CW");

        // Get track width for angular velocity calculation
        // You'll need to pass this in or access it somehow
        // double track_width_inches = 14.0; // Your robot's track width

        for (double voltage = 0.0; voltage <= max_voltage; voltage += voltage_step)
        {
            uint32_t step_start = pros::millis();

            // Apply opposite voltages to wheels
            units::Voltage left_voltage = units::Voltage::from_volts(voltage * left_multiplier);
            units::Voltage right_voltage = units::Voltage::from_volts(voltage * right_multiplier);

            left.move_voltage(left_voltage);
            right.move_voltage(right_voltage);

            while ((pros::millis() - step_start) < static_cast<uint32_t>(step_duration_ms))
            {
                units::MotorAngularVelocity left_vel = left.get_raw_velocity();
                units::MotorAngularVelocity right_vel = right.get_raw_velocity();

                // Average wheel speed magnitude (for feedforward)
                double avg_wheel_vel = (std::abs(left_vel.rad_per_sec) +
                                        std::abs(right_vel.rad_per_sec)) /
                                       2.0;

                // Robot angular velocity: omega = (v_right - v_left) / track_width
                // Note: velocities already have correct signs from multipliers
                double angular_vel = (right_vel.rad_per_sec - left_vel.rad_per_sec);
                // If you want body angular velocity: / track_width_inches

                uint32_t timestamp = pros::millis() - test_start_time;

                fprintf(file, "%lu,%.2f,%.4f,%.4f,%.4f,%.4f\n",
                        timestamp, voltage,
                        left_vel.rad_per_sec, right_vel.rad_per_sec,
                        avg_wheel_vel, angular_vel);

                pros::delay(10);
            }

            if (static_cast<int>(voltage) % 2 == 0)
            {
                pros::lcd::print(1, "Testing %.1fV...", voltage);
            }
        }
        left.move_voltage(units::Voltage::from_volts(0));
        right.move_voltage(units::Voltage::from_volts(0));

        left.brake();
        right.brake();
        fclose(file);

        pros::lcd::print(0, "Turn test complete!");
        pros::lcd::print(1, "File: %s", full_filename);
        pros::lcd::print(2, "Analyze for turn");
        pros::lcd::print(3, "kS_turn and kV_turn");
    }

    void measure_velocity_pid(
        hardware::Chassis &chassis, // Changed from left/right motor groups
        bool forward,
        const char *filename,
        double target_rpm,
        int settle_duration_ms)
    {
        // Check SD card availability
        if (!pros::usd::is_installed())
        {
            pros::lcd::print(0, "ERROR: No SD card!");
            return;
        }

        // Build filename with velocity and direction
        char full_filename[64];
        snprintf(full_filename, sizeof(full_filename), "/usd/%s_%.0frpm_%s.csv",
                 filename, target_rpm, forward ? "forward" : "backward");

        pros::lcd::print(0, "Starting velocity PID test...");
        pros::lcd::print(1, "Target: %.1f RPM", target_rpm);
        pros::delay(2000);

        // Reset chassis position instead of individual motor groups
        chassis.reset_chassis_position();
        pros::delay(100);

        // Open file for writing
        FILE *file = fopen(full_filename, "w");
        if (file == nullptr)
        {
            pros::lcd::print(0, "ERROR: Can't open file!");
            return;
        }

        // Write CSV header
        fprintf(file, "time_ms,target_rpm,left_vel_rpm,right_vel_rpm,avg_vel_rpm,");
        fprintf(file, "error_rpm,left_p,left_i,left_d,right_p,right_i,right_d\n");

        uint32_t test_start_time = pros::millis();
        double direction_multiplier = forward ? 1.0 : -1.0;
        units::MotorAngularVelocity target_velocity =
            units::MotorAngularVelocity::from_rpm(target_rpm * direction_multiplier);

        pros::lcd::print(0, "Holding %.1f RPM...", target_rpm * direction_multiplier);

        // Hold velocity for the specified duration
        while ((pros::millis() - test_start_time) < static_cast<uint32_t>(settle_duration_ms))
        {
            // Convert motor velocity to wheel velocity
            units::Distance wheel_radius = chassis.get_config().diameter / 2.0;
            units::WheelLinearVelocity wheel_vel =
                units::WheelLinearVelocity(target_velocity.rad_per_sec * wheel_radius.inches);

            chassis.move_velocity(wheel_vel, wheel_vel);

            // Read from telemetry
            TelemetryData local_telem;
            {
                std::lock_guard<pros::Mutex> lock(telemetry_mutex);
                local_telem = telemetry;
            }

            // Convert velocities to RPM for logging
            double left_vel_rpm = units::RPM::from_rad_per_sec(
                                      local_telem.left_motor_actual_velocity.rad_per_sec)
                                      .value;
            double right_vel_rpm = units::RPM::from_rad_per_sec(
                                       local_telem.right_motor_actual_velocity.rad_per_sec)
                                       .value;
            double avg_vel_rpm = (left_vel_rpm + right_vel_rpm) / 2.0;

            // Average error
            double avg_error_rpm = (local_telem.left_motor_velocity_error_rpm +
                                    local_telem.right_motor_velocity_error_rpm) /
                                   2.0;

            uint32_t timestamp = pros::millis() - test_start_time;

            // Write to SD card with all PID terms
            fprintf(file, "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    timestamp,
                    target_rpm * direction_multiplier,
                    left_vel_rpm,
                    right_vel_rpm,
                    avg_vel_rpm,
                    avg_error_rpm,
                    local_telem.left_motor_velocity_p_term,
                    local_telem.left_motor_velocity_i_term,
                    local_telem.left_motor_velocity_d_term,
                    local_telem.right_motor_velocity_p_term,
                    local_telem.right_motor_velocity_i_term,
                    local_telem.right_motor_velocity_d_term);

            // Update LCD every 500ms
            if (timestamp % 500 < 10)
            {
                pros::lcd::print(1, "Vel: %.1f RPM", avg_vel_rpm);
                pros::lcd::print(2, "Err: %.1f RPM", avg_error_rpm);
            }

            pros::delay(10); // Sample at 100Hz
        }

        // Stop motors through chassis
        chassis.stop_motors();
        fclose(file);

        pros::lcd::print(0, "Velocity test complete!");
        pros::lcd::print(1, "File: %s", full_filename);
        pros::lcd::print(2, "Analyze PID response");
    }
}