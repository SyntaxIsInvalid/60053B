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
        //robot.chassis.drive_straight_relative(18_in, 5_s);
        //robot.chassis.turn_to_heading_profiled_pid(90_deg, 90_rad_per_sec, 180_rad_per_sec2, 10_s);
        /*
control::PurePursuitConfig config;
config.target_velocity = units::Velocity::from_ips(36.0);
config.max_acceleration = units::Acceleration::from_ips2(60.0);
config.use_motion_profile = true;
config.use_adaptive_lookahead = true;
config.use_heading_correction = false;
config.heading_correction_gain = 3;
config.heading_start_threshold = 0.6;
config.use_final_turn = true;
        robot.chassis.quintic_pure_pursuit(
            units::Length::from_inches(24.0), // target x
            units::Length::from_inches(24.0), // target y
            units::Angle::from_degrees(90),   // target heading
            config,
            units::Time::from_seconds(10)     // timeout
        );
        */
        //robot.chassis.quintic_ramsete(24_in, 24_in, 90_deg, 18_ips, 36_ips2, 10_s);
    }
}