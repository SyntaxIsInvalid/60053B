#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void solo_awp_red(RobotSubsystems &robot)
    {
        // Set starting position (alliance corner frame)
        //robot.chassis.set_pose(89.5_in, 28.5_in, -90_deg);
        robot.chassis.set_pose(0_in, 0_in, -90_deg);
        robot.chassis.drive_straight_relative(38_in, 5_s);
        robot.match_load_ramp.extend();
        robot.chassis.turn_to_heading(180_deg, 5_s);
        robot.chassis.drive_straight_relative(10_in, 5_s);
        pros::delay(500);
    }
}