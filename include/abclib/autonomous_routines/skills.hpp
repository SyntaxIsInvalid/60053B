#pragma once
#include "auton_selector.hpp"

using namespace abclib::units::literals;

namespace abclib::auton
{
    inline void skills(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.set_pose(0_in, 0_in, 0_deg);
    }
}