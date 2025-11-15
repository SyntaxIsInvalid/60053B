#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void test_bot_auton(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.move_straight_profiled_pid(
            units::Length::from_inches(10.0),
            units::Velocity::from_ips(10.0),
            units::Acceleration::from_mps2(50.0),
            units::Time::from_seconds(3.0));
    }
}