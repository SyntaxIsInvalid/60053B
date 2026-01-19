#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void solo_awp_red(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.set_pose(0_in, 0_in, 0_deg);
        robot.chassis.turn_to_heading_profiled_pid(90_deg, 180_deg_per_sec, 360_deg_per_sec2, 5_s);
    }
}