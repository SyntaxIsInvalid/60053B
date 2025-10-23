#pragma once

#include "abclib/hardware/chassis.hpp"
#include <vector>

namespace abclib::characterization
{
    void measure_lateral_pid(
        hardware::Chassis &chassis,
        const std::vector<double> &target_distances = {},
        const char *filename = "lateral_pid_test",
        int settle_delay_ms = 500);

    void measure_angular_pid(
        hardware::Chassis &chassis,
        const std::vector<double> &target_headings = {},
        const char *filename = "angular_pid_test",
        int settle_delay_ms = 500);

}