#pragma once
#include "auton_selector.hpp"

namespace abclib::auton {
    inline void none(RobotSubsystems& robot) {
        // Set starting position
        robot.chassis.set_pose(
            units::Distance::from_inches(0),
            units::Distance::from_inches(0),
            units::Degrees(0));
        robot.chassis.drive_straight_relative(units::Distance(6), units::Time::from_seconds(3));
    }
}