#pragma once
#include "auton_selector.hpp"
#include "abclib/telemetry/path_logger.hpp"
namespace abclib::auton
{
    inline void solo_awp_red(RobotSubsystems &robot)
    {
        robot.chassis.set_pose(0_in, 0_in, -90_deg);
        robot.chassis.drive_straight_relative(35_in, 1.5_s);
        robot.wing.extend();
        robot.match_load_ramp.extend();
        robot.chassis.turn_to_heading(180_deg, 1_s);
        robot.bottom_intake.set_intake();
        robot.chassis.drive_straight_relative(11.5_in, 1.5_s);
        robot.chassis.move_voltage(3_V, 3_V);
        pros::delay(650);
        robot.chassis.stop_motors();
        
        robot.chassis.drive_straight_relative(-10_in, 1.5_s);
        robot.top_intake.set_voltage(-2_V);
        robot.chassis.turn_to_heading(135_deg, 1_s);
        robot.chassis.drive_straight_relative(-18_in, 1.5_s);
        robot.chassis.turn_to_heading(180_deg, 1_s);
        robot.match_load_ramp.retract();
        robot.bottom_intake.set_idle();
        robot.chassis.drive_straight_relative(-66_in, 2_s);
        robot.chassis.turn_to_heading(235_deg, 1.5_s);
        robot.chassis.drive_straight_relative(-13_in, 1_s);
        robot.chassis.turn_to_heading(0_deg, 1.2_s);
        robot.chassis.move_voltage(-7_V, -7_V);
        pros::delay(1000);
        robot.hood.retract();
        robot.bottom_intake.set_intake();
        robot.top_intake.set_intake();
        robot.chassis.stop_motors();
        pros::delay(1500);
        robot.bottom_intake.set_idle();
        robot.top_intake.set_idle();
        robot.match_load_ramp.extend();
        robot.chassis.drive_straight_relative(24_in, 1_s, 0_V, 4_V);
        robot.chassis.move_voltage(3_V, 3_V);
        pros::delay(750);
        robot.bottom_intake.set_intake();
        pros::delay(1000);
        robot.chassis.stop_motors();
        robot.chassis.move_voltage(-8_V, -8_V);
        pros::delay(1500);
        robot.chassis.stop_motors();
        robot.top_intake.set_intake();
        pros::delay(1500);
        robot.chassis.drive_straight_relative(18_in, 1.5_s);
        robot.match_load_ramp.retract();
        robot.chassis.turn_to_heading(45_deg, 1_s);
        robot.chassis.drive_straight_relative(8_in, 1.5_s);
                     robot.chassis.turn_to_heading(90_deg, 1_s);
                     robot.match_load_ramp.extend();  
                     pros::delay(250);
        robot.chassis.drive_straight_relative(20_in, 5_s);
        //robot.match_load_ramp.extend();
        //robot.chassis.set_pose(0_in, 5_in, 0_deg);
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