#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void test_bot_auton(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.set_pose(0_in, 0_in, 0_deg);
        robot.chassis.move_straight_profiled(10_in, 20_ips, 50_ips2, 5_s);
        /*
        robot.chassis.move_straight_profiled_pid(
            10_in, 20_ips, 50_ips2, 5_s
        );
        */
        /*
        robot.chassis.turn_to_heading_profiled_pid(
            units::Angle::from_degrees(90.0),
            units::AngularVelocity::from_deg_per_sec(180),
            units::AngularAcceleration::from_deg_per_sec2(360),
            units::Time::from_seconds(5));
        */
    }
}