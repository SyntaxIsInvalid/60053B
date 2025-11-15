#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void test_bot_auton(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.turn_to_heading_profiled_pid(
            units::Angle::from_degrees(90.0),
            units::AngularVelocity::from_deg_per_sec(180.0),
            units::AngularAcceleration::from_deg_per_sec2(360.0),
            units::Time::from_seconds(2.0));
    }
}