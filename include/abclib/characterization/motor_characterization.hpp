#pragma once

#include "abclib/hardware/motor_group.hpp"

namespace abclib::hardware
{
    class Chassis;
}

namespace abclib::characterization
{
    void measure_ks_kv(
        hardware::AdvancedMotorGroup &left,
        hardware::AdvancedMotorGroup &right,
        bool forward = true,
        const char *filename = "ks_kv_test",
        double max_voltage = 5,
        double voltage_step = 0.1,
        int step_duration_ms = 100);

    void measure_ks_kv_turn(
        hardware::AdvancedMotorGroup &left,
        hardware::AdvancedMotorGroup &right,
        bool ccw_rotation = true,
        const char *filename = "ks_kv_turn_test",
        double max_voltage = 10,
        double voltage_step = 0.25,
        int step_duration_ms = 500);

    void measure_velocity_pid(
        hardware::Chassis &chassis,
        bool forward,
        const char *filename,
        double target_rpm,
        int settle_duration_ms);
}