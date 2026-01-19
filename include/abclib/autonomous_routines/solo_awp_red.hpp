#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void solo_awp_red(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.set_pose(0_in, 0_in, 0_deg);
        robot.chassis.drive_straight_relative(12_in, 1_s);
    }
}