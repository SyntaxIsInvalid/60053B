#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void path_builder_test(RobotSubsystems &robot)
    {
    builder::PathBuilder builder(14.0_in);  // 14 inch track width

    path::Path test_path = builder
        .start(0.0_in, 0.0_in, 0.0_rad)
        
        // Profile 1: Fast approach with chained splines
        .begin_profile("fast_approach")
            .trapezoidal(48.0_ips, 72.0_ips2)
        .spline_to(24.0_in, 12.0_in, 45.0_deg)
        .spline_to(36.0_in, 24.0_in, 90.0_deg)
        .straight_forward(12.0_in)
        
        // Profile 2: Turn (dedicated profile, must be empty)
        .begin_profile("turn_around")
            .trapezoidal(24.0_ips, 36.0_ips2)
        .turn_in_place(180.0_deg)
        
        // Profile 3: Slow precision
        .begin_profile("slow_precision")
            .trapezoidal(24.0_ips, 36.0_ips2)
        .straight_forward(6.0_in)
        
        .break_continuity()
        
        // Profile 4: Return with explicit spline control
        .begin_profile("return_home")
            .trapezoidal(60.0_ips, 80.0_ips2)
        .quintic_hermite()
            .to(12.0_in, 18.0_in, -45.0_deg)
            .with_tangent(15.0_in, 0.0_in)
            .with_second_derivative(0.0_in, 0.0_in)
            .end_tangent(10.0_in, -10.0_in)
            .end_second_derivative(0.0_in, 0.0_in)
            .build()
        .spline_to(0.0_in, 0.0_in, 0.0_rad)
        
        .build();

        //builder::PathLogger logger("test_path", 14.0_in);
        // logger.log_all(test_path);


        /*
        // Add path:: and trajectory:: qualifiers
        path::PathLogger::log_path(test_complex, "test_complex_path");
        trajectory::TrajectoryLogger::log_path_trajectories(test_complex, "test_complex_trajectories");
        pros::lcd::print(0, "All path tests logged!");
        */
    }
}