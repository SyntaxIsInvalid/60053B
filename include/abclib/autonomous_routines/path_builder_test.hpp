#pragma once
#include "auton_selector.hpp"

namespace abclib::auton {
    inline void path_builder_test(RobotSubsystems& robot) {
        /*
        path::PathBuilder builder(units::Distance::from_inches(14.0));
        path::Path test_complex = builder
            .start(0, 0, 0)
            .begin_profile("approach",
                        units::BodyLinearVelocity(36.0),
                        3.0)
            .spline_to(24, 12, M_PI/6)
            .straight_forward(20.0)
            .begin_profile("turn1",
                        units::BodyLinearVelocity(24.0),
                        2.0)
            .turn_in_place(M_PI/2)
            .begin_profile("pickup",
                        units::BodyLinearVelocity(12.0),
                        1.0)
            .spline_to(48, 48, M_PI, {{20.0, 20.0, 1.0, 1.0, 0.0, 0.0}})
            .break_continuity()
            .begin_profile("return",
                        units::BodyLinearVelocity(36.0),
                        3.0)
            .spline_to(24, 24, -M_PI/4)
            .begin_profile("turn2",
                        units::BodyLinearVelocity(20.0),
                        2.5)
            .turn_in_place(-3.0 * M_PI / 4.0)
            .begin_profile("final_approach",
                        units::BodyLinearVelocity(36.0),
                        3.0)
            .straight_to(0, 0)
            .build();

        // Add path:: and trajectory:: qualifiers
        path::PathLogger::log_path(test_complex, "test_complex_path");
        trajectory::TrajectoryLogger::log_path_trajectories(test_complex, "test_complex_trajectories");
        pros::lcd::print(0, "All path tests logged!");
        */
    }
}