#pragma once
#include "auton_selector.hpp"
#include "abclib/telemetry/path_logger.hpp"
namespace abclib::auton
{
    inline void solo_awp_red(RobotSubsystems &robot)
    {
        robot.chassis.set_pose(0_in, 5_in, 0_deg);
        /*
        // Set starting position (alliance corner frame)
        robot.chassis.set_pose(0_in, 0_in, 90_deg);
        estimation::Pose start = robot.chassis.get_pose();

        units::Length target_x = units::Length::from_inches(24.0);
        units::Length target_y = units::Length::from_inches(24.0);
        units::Angle target_heading = units::Angle::from_degrees(0.0);

        path::Pose start_pose(
            start.x_inches(),
            start.y_inches(),
            start.theta_rad());

        path::Pose end_pose(
            target_x.to_inches(),
            target_y.to_inches(),
            target_heading.to_radians());

        // Build the path
        path::Path quintic_path;
        path::ProfileGroup group(
            "test_quintic",
            units::Velocity::from_ips(24.0),
            units::Acceleration::from_ips2(48.0));

        auto segment = std::make_unique<path::QuinticHermiteSegment>(
            start_pose, end_pose);
        group.add_segment(std::move(segment));
        group.compute_arc_length();

        quintic_path.add_profile_group(std::move(group));

        // Log to CSV BEFORE following the path
        telemetry::PathLogger::log_profile_group_to_csv(
            quintic_path.get_profile_groups()[0],
            "/usd/quintic_spline.csv", // Saves to SD card
            100,                       // 100 samples
            robot.chassis.get_alliance(),
            robot.chassis.get_config().field_config);
        */
    }
}