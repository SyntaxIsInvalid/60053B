#include "abclib/sysid/motor_characterization.hpp"
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/chassis.hpp"
#include "abclib/units/units.hpp"
#include "api.h"
#include <fstream>
#include <vector>
#include <cmath>

namespace abclib::sysid
{
    using namespace units;

    void measure_ks_kv(
        hardware::AdvancedMotorGroup &left,
        hardware::AdvancedMotorGroup &right,
        bool forward,
        const char *filename,
        double max_voltage,
        double voltage_step,
        int step_duration_ms)
    {
        std::ofstream file(filename);
        file << "voltage,left_velocity_rpm,right_velocity_rpm,avg_velocity_rpm\n";

        left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        double direction = forward ? 1.0 : -1.0;

        for (double voltage = 0.0; voltage <= max_voltage; voltage += voltage_step)
        {
            Voltage applied_voltage = Voltage::from_volts(direction * voltage);

            left.move_voltage(applied_voltage);
            right.move_voltage(applied_voltage);

            pros::delay(step_duration_ms);

            AngularVelocity left_vel = left.get_raw_velocity();
            AngularVelocity right_vel = right.get_raw_velocity();

            double left_rpm = left_vel.to_rpm();
            double right_rpm = right_vel.to_rpm();
            double avg_rpm = (left_rpm + right_rpm) / 2.0;

            file << voltage << ","
                 << left_rpm << ","
                 << right_rpm << ","
                 << avg_rpm << "\n";
        }

        left.brake();
        right.brake();
        file.close();
    }

    void measure_ks_kv_turn(
        hardware::AdvancedMotorGroup &left,
        hardware::AdvancedMotorGroup &right,
        bool ccw_rotation,
        const char *filename,
        double max_voltage,
        double voltage_step,
        int step_duration_ms)
    {
        std::ofstream file(filename);
        file << "voltage,left_velocity_rpm,right_velocity_rpm,avg_velocity_rpm\n";

        left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        double direction = ccw_rotation ? 1.0 : -1.0;

        for (double voltage = 0.0; voltage <= max_voltage; voltage += voltage_step)
        {
            Voltage left_voltage = Voltage::from_volts(direction * voltage);
            Voltage right_voltage = Voltage::from_volts(-direction * voltage);

            left.move_voltage(left_voltage);
            right.move_voltage(right_voltage);

            pros::delay(step_duration_ms);

            AngularVelocity left_vel = left.get_raw_velocity();
            AngularVelocity right_vel = right.get_raw_velocity();

            double left_rpm = left_vel.to_rpm();
            double right_rpm = right_vel.to_rpm();
            double avg_rpm = (std::abs(left_rpm) + std::abs(right_rpm)) / 2.0;

            file << voltage << ","
                 << left_rpm << ","
                 << right_rpm << ","
                 << avg_rpm << "\n";
        }

        left.brake();
        right.brake();
        file.close();
    }

    void measure_velocity_pid(
        hardware::Chassis &chassis,
        bool forward,
        const char *filename,
        double target_rpm,
        int settle_duration_ms)
    {
        std::ofstream file(filename);
        file << "time_ms,target_rpm,left_rpm,right_rpm,left_voltage,right_voltage\n";

        AngularVelocity target_velocity = AngularVelocity::from_rpm(forward ? target_rpm : -target_rpm);

        uint32_t start_time = pros::millis();
        uint32_t current_time = 0;

        // Get the wheel radius to convert linear velocity to angular velocity
        Length wheel_radius = chassis.get_wheel_radius();

        // Convert RPM to linear velocity (ips) then back to angular velocity for the motors
        Velocity linear_velocity = Velocity::from_ips(
            (target_velocity.to_rpm() / 60.0) * 2.0 * constants::PI * wheel_radius.to_inches());

        while (current_time < static_cast<uint32_t>(settle_duration_ms))
        {
            current_time = pros::millis() - start_time;

            // Use the chassis move_velocity method directly
            chassis.move_velocity_pros(linear_velocity, linear_velocity);

            // Get velocities from telemetry
            const auto &telem = telemetry::g_telemetry.get_read_buffer();
            double left_rpm = telem.left_motor_actual_velocity.to_rpm();
            double right_rpm = telem.right_motor_actual_velocity.to_rpm();

            file << current_time << ","
                 << target_rpm << ","
                 << left_rpm << ","
                 << right_rpm << ",0,0\n"; // Voltage values from telemetry if needed

            pros::delay(10);
        }

        chassis.stop_motors();
        file.close();
    }

    void measure_ka(
        hardware::AdvancedMotorGroup &left,
        hardware::AdvancedMotorGroup &right,
        bool forward,
        const char *filename,
        double constant_voltage,
        int test_duration_ms)
    {
        std::ofstream file(filename);
        file << "time_ms,left_velocity_rpm,right_velocity_rpm,left_acceleration,right_acceleration\n";

        left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        left.reset_position();
        right.reset_position();

        double direction = forward ? 1.0 : -1.0;
        Voltage applied_voltage = Voltage::from_volts(direction * constant_voltage);

        uint32_t start_time = pros::millis();
        AngularVelocity prev_left_vel = AngularVelocity::from_rpm(0.0);
        AngularVelocity prev_right_vel = AngularVelocity::from_rpm(0.0);
        uint32_t prev_time = start_time;

        left.move_voltage(applied_voltage);
        right.move_voltage(applied_voltage);

        while ((pros::millis() - start_time) < static_cast<uint32_t>(test_duration_ms))
        {
            uint32_t current_time = pros::millis();
            double dt = (current_time - prev_time) / 1000.0;

            AngularVelocity left_vel = left.get_raw_velocity();
            AngularVelocity right_vel = right.get_raw_velocity();

            double left_accel = 0.0;
            double right_accel = 0.0;

            if (dt > 0.0)
            {
                left_accel = (left_vel.to_rad_per_sec() - prev_left_vel.to_rad_per_sec()) / dt;
                right_accel = (right_vel.to_rad_per_sec() - prev_right_vel.to_rad_per_sec()) / dt;
            }

            file << (current_time - start_time) << ","
                 << left_vel.to_rpm() << ","
                 << right_vel.to_rpm() << ","
                 << left_accel << ","
                 << right_accel << "\n";

            prev_left_vel = left_vel;
            prev_right_vel = right_vel;
            prev_time = current_time;

            pros::delay(10);
        }

        left.brake();
        right.brake();
        file.close();
    }

    void measure_ka_turn(
        hardware::AdvancedMotorGroup &left,
        hardware::AdvancedMotorGroup &right,
        bool ccw_rotation,
        const char *filename,
        double constant_voltage,
        int test_duration_ms)
    {
        std::ofstream file(filename);
        file << "time_ms,left_velocity_rpm,right_velocity_rpm,left_acceleration,right_acceleration\n";

        left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        left.reset_position();
        right.reset_position();

        double direction = ccw_rotation ? 1.0 : -1.0;
        Voltage left_voltage = Voltage::from_volts(direction * constant_voltage);
        Voltage right_voltage = Voltage::from_volts(-direction * constant_voltage);

        uint32_t start_time = pros::millis();
        AngularVelocity prev_left_vel = AngularVelocity::from_rpm(0.0);
        AngularVelocity prev_right_vel = AngularVelocity::from_rpm(0.0);
        uint32_t prev_time = start_time;

        left.move_voltage(left_voltage);
        right.move_voltage(right_voltage);

        while ((pros::millis() - start_time) < static_cast<uint32_t>(test_duration_ms))
        {
            uint32_t current_time = pros::millis();
            double dt = (current_time - prev_time) / 1000.0;

            AngularVelocity left_vel = left.get_raw_velocity();
            AngularVelocity right_vel = right.get_raw_velocity();

            double left_accel = 0.0;
            double right_accel = 0.0;

            if (dt > 0.0)
            {
                left_accel = (left_vel.to_rad_per_sec() - prev_left_vel.to_rad_per_sec()) / dt;
                right_accel = (right_vel.to_rad_per_sec() - prev_right_vel.to_rad_per_sec()) / dt;
            }

            file << (current_time - start_time) << ","
                 << left_vel.to_rpm() << ","
                 << right_vel.to_rpm() << ","
                 << left_accel << ","
                 << right_accel << "\n";

            prev_left_vel = left_vel;
            prev_right_vel = right_vel;
            prev_time = current_time;

            pros::delay(10);
        }

        left.brake();
        right.brake();
        file.close();
    }

} // namespace abclib::sysid