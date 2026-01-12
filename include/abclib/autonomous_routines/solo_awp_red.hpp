#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void solo_awp_red(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.set_pose(
            units::Length::from_inches(0),
            units::Length::from_inches(0),
            units::Angle::from_degrees(0));

        // robot.chassis.follow_path_pure_pursuit(my_path, )
control::PurePursuitConfig config;
config.target_velocity = units::Velocity::from_ips(18.0);
config.max_acceleration = units::Acceleration::from_ips2(36.0);
config.use_motion_profile = true;
config.use_adaptive_lookahead = true;

        robot.chassis.quintic_pure_pursuit(
            units::Length::from_inches(24.0), // target x
            units::Length::from_inches(24.0), // target y
            units::Angle::from_degrees(90),   // target heading
            config,
            units::Time::from_seconds(10)     // timeout
        );
    }
}