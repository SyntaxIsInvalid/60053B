#pragma once
#include "auton_selector.hpp"

namespace abclib::auton {
    inline void solo_awp_red(RobotSubsystems& robot) {
        // Set starting position
        robot.chassis.set_pose(
            units::Distance::from_inches(0),
            units::Distance::from_inches(0),
            units::Degrees(0));
        
        robot.top_intake.set_intake();
    }
}