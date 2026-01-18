#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void solo_awp_red(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.set_pose(4.5_in, 12_in, 0_deg);
        robot.chassis.configure_distance_correction(
            3.35_in, // sensor is 1 inch forward from tracking center
            0_in, // sensor is 2 inches to the left
            0.3   // 30% sensor, 70% odometry
        );
        robot.chassis.enable_distance_correction(true);
    }
}