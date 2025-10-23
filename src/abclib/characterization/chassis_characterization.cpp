#include "abclib/characterization/chassis_characterization.hpp"
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/units/units.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.h"
#include <cstdio>
#include <mutex>

namespace abclib::characterization
{

    void measure_lateral_pid(
        hardware::Chassis &chassis,
        const std::vector<double> &target_distances,
        const char *filename,
        int settle_delay_ms)
    {
        // Check SD card availability
        if (!pros::usd::is_installed())
        {
            pros::lcd::print(0, "ERROR: No SD card!");
            return;
        }

        // Use default sequence if none provided
        std::vector<double> distances = target_distances.empty()
                                            ? std::vector<double>{0, 18, 6, 36, 12, -12, 6, 0}
                                            : target_distances;

        // Build filename
        char full_filename[64];
        snprintf(full_filename, sizeof(full_filename), "/usd/%s.csv", filename);

        pros::lcd::print(0, "Lateral PID Test");
        pros::lcd::print(1, "Steps: %d", static_cast<int>(distances.size()));
        pros::delay(2000);

        // Reset chassis position
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
        fprintf(file, "time_ms,target_dist,actual_dist,dist_error,lateral_p,lateral_i,lateral_d,");
        fprintf(file, "angular_error,angular_p,angular_i,angular_d,left_voltage,right_voltage,");
        fprintf(file, "velocity,angular_velocity,settled,settlement_reason\n");

        uint32_t test_start_time = pros::millis();

        // Execute each step in sequence
        for (size_t step = 0; step < distances.size(); ++step)
        {
            double target_dist = distances[step];
            pros::lcd::print(0, "Step %d/%d", step + 1, distances.size());
            pros::lcd::print(1, "Target: %.1f in", target_dist);

            // Start logging task
            bool logging_active = true;
            pros::Task log_task([&]()
                                {
                uint32_t step_start = pros::millis();
                while (logging_active) {
                    // Thread-safe telemetry copy
                    TelemetryData local_telem;
                    {
                        std::lock_guard<pros::Mutex> lock(telemetry_mutex);
                        local_telem = telemetry;
                    }

                    uint32_t timestamp = pros::millis() - test_start_time;

                    // Write data row
                    fprintf(file, "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                            timestamp,
                            target_dist,
                            local_telem.lateral_actual.inches,
                            local_telem.lateral_error.inches,
                            local_telem.lateral_p_term,
                            local_telem.lateral_i_term,
                            local_telem.lateral_d_term);

                    fprintf(file, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                            local_telem.angular_error.value,
                            local_telem.angular_p_term,
                            local_telem.angular_i_term,
                            local_telem.angular_d_term,
                            local_telem.left_motor_voltage.volts,
                            local_telem.right_motor_voltage.volts);

                    fprintf(file, "%.3f,%.3f,%d,%s\n",
                            local_telem.pose_v.inches_per_sec,
                            local_telem.pose_omega.rad_per_sec,
                            local_telem.is_settled ? 1 : 0,
                            settlement_reason_to_string(local_telem.settlement_reason));

                    pros::delay(10); // 100Hz logging
                } });

            // Execute movement
            chassis.drive_straight_relative(
                units::Distance::from_inches(target_dist),
                units::Time::from_seconds(10),
                units::Voltage::from_volts(0),
                units::Voltage::from_volts(12),
                units::Voltage::from_volts(0),
                units::Voltage::from_volts(6),
                false); // Don't reset position between steps

            // Stop logging
            logging_active = false;
            pros::delay(20); // Let task finish
            log_task.remove();

            // Settling delay between steps
            if (step < distances.size() - 1)
            {
                pros::lcd::print(2, "Settling...");
                pros::delay(settle_delay_ms);
            }
        }

        fclose(file);

        pros::lcd::print(0, "Lateral test complete!");
        pros::lcd::print(1, "File: %s", filename);
        pros::lcd::print(2, "Analyze for PID tuning");
    }

    void measure_angular_pid(
        hardware::Chassis &chassis,
        const std::vector<double> &target_headings,
        const char *filename,
        int settle_delay_ms)
    {
        // Check SD card availability
        if (!pros::usd::is_installed())
        {
            pros::lcd::print(0, "ERROR: No SD card!");
            return;
        }

        // Use default sequence if none provided
        std::vector<double> headings = target_headings.empty()
                                           ? std::vector<double>{0, 90, 45, 135, -45, -90, 0}
                                           : target_headings;

        // Build filename
        char full_filename[64];
        snprintf(full_filename, sizeof(full_filename), "/usd/%s.csv", filename);

        pros::lcd::print(0, "Angular PID Test");
        pros::lcd::print(1, "Steps: %d", static_cast<int>(headings.size()));
        pros::delay(2000);

        // Reset chassis position
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
        fprintf(file, "time_ms,target_heading,actual_heading,angular_error,angular_p,angular_i,angular_d,");
        fprintf(file, "left_voltage,right_voltage,angular_velocity,settled,settlement_reason\n");

        uint32_t test_start_time = pros::millis();

        // Execute each step in sequence
        for (size_t step = 0; step < headings.size(); ++step)
        {
            double target_heading = headings[step];
            pros::lcd::print(0, "Step %d/%d", step + 1, headings.size());
            pros::lcd::print(1, "Target: %.1f deg", target_heading);

            // Start logging task
            bool logging_active = true;
            pros::Task log_task([&]()
                                {
                while (logging_active) {
                    // Thread-safe telemetry copy
                    TelemetryData local_telem;
                    {
                        std::lock_guard<pros::Mutex> lock(telemetry_mutex);
                        local_telem = telemetry;
                    }

                    uint32_t timestamp = pros::millis() - test_start_time;

                    // Write data row
                    fprintf(file, "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
                            timestamp,
                            target_heading,
                            local_telem.angular_actual.to_degrees().value,
                            local_telem.angular_error.to_degrees().value,
                            local_telem.angular_p_term,
                            local_telem.angular_i_term,
                            local_telem.angular_d_term);

                    fprintf(file, "%.3f,%.3f,%.3f,%d,%s\n",
                            local_telem.left_motor_voltage.volts,
                            local_telem.right_motor_voltage.volts,
                            local_telem.pose_omega.rad_per_sec,
                            local_telem.is_settled ? 1 : 0,
                            settlement_reason_to_string(local_telem.settlement_reason));

                    pros::delay(10); // 100Hz logging
                } });

            // Execute turn
            chassis.turn_to_heading(
                units::Degrees(target_heading),
                units::Time::from_seconds(5),
                units::Voltage::from_volts(0),
                units::Voltage::from_volts(6),
                false); // Don't reset position between steps

            // Stop logging
            logging_active = false;
            pros::delay(20); // Let task finish
            log_task.remove();

            // Settling delay between steps
            if (step < headings.size() - 1)
            {
                pros::lcd::print(2, "Settling...");
                pros::delay(settle_delay_ms);
            }
        }

        fclose(file);

        pros::lcd::print(0, "Angular test complete!");
        pros::lcd::print(1, "File: %s", filename);
        pros::lcd::print(2, "Analyze for PID tuning");
    }

} // namespace abclib::characterization