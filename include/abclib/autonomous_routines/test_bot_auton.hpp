#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void test_bot_auton(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.set_pose(
            units::Distance::from_inches(0),
            units::Distance::from_inches(0),
            units::Degrees(0));
        robot.chassis.turn_to_heading_profiled_pid(
            units::Degrees(90.0),
            180,
            360,
            units::Time::from_seconds(10.0));
    }
}