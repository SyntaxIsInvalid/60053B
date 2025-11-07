#pragma once
#include "auton_selector.hpp"

namespace abclib::auton {
    inline void skills(RobotSubsystems& robot) {
        // Set starting position
        robot.chassis.set_pose(
            units::Distance::from_inches(0),
            units::Distance::from_inches(0),
            units::Degrees(0));

    }
}