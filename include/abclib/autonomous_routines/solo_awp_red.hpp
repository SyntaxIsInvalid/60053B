#pragma once
#include "auton_selector.hpp"
#include "abclib/telemetry/path_logger.hpp"
namespace abclib::auton
{
    inline void solo_awp_red(RobotSubsystems &robot)
    {
        robot.chassis.set_pose(0_in, 0_in, -90_deg);
        robot.hood.retract();
        robot.chassis.drive_straight_relative(28_in, 1_s);
        robot.wing.extend();
        robot.match_load_ramp.extend();
        robot.chassis.turn_to_heading(180_deg, 1.5_s);
        robot.bottom_intake.set_intake();
        robot.chassis.drive_straight_relative(10_in, .85_s);
        robot.chassis.move_voltage(2_V, 2_V);
        robot.chassis.stop_motors();
        robot.chassis.move_voltage(-8_V, -8_V);

        pros::delay(1500);
        robot.bottom_intake.set_intake();
        robot.top_intake.set_intake();
        pros::delay(1000);
        robot.bottom_intake.set_idle();
        robot.top_intake.set_idle();
        robot.chassis.stop_motors();
        auto pose = robot.chassis.get_pose();
        robot.chassis.set_pose(pose.x(), pose.y(), 180_deg);
        robot.match_load_ramp.retract();
        robot.chassis.drive_straight_relative(17_in, 0.8_s);
        robot.chassis.turn_to_heading(45_deg, 0.7_s);
        robot.bottom_intake.set_intake();
        robot.chassis.drive_straight_relative(33.941125497_in, 1.5_s);
        robot.chassis.turn_to_heading(90_deg, 0.5_s);
        robot.chassis.drive_straight_relative(46_in, 2_s);
        robot.chassis.turn_to_heading(135_deg, 1_s);
        robot.chassis.drive_straight_relative(-15_in, 1_s);
        robot.chassis.turn_to_heading(135_deg, 0.3_s);
        robot.mid_goal.extend();
        robot.bottom_intake.intake_for_voltage(-12_V, .01_s);
        robot.top_intake.intake_for_voltage(-12_V, 1_s);
        robot.bottom_intake.set_intake();
        pros::delay(1000);
        robot.chassis.drive_straight_relative(47_in, 1_s);
        robot.mid_goal.retract();
        robot.match_load_ramp.extend();
        robot.chassis.turn_to_heading(180_deg, 1_s);
        robot.top_intake.set_idle();
        robot.bottom_intake.set_intake();
        robot.chassis.drive_straight_relative(11_in, .8_s);
        pros::delay(500);
        robot.chassis.move_voltage(-8_V, -8_V);
        pros::delay(1500);
        robot.top_intake.set_intake();

        // robot.chassis.set_pose(0_in, 5_in, 0_deg);
        /*
        robot.chassis.set_pose(70_in, 23.11_in, -90_deg);
        robot.chassis.drive_straight_relative(38_in, 1.5_s);
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
                robot.bottom_intake.set_idle();
        robot.chassis.drive_straight_relative(-18_in, 1.5_s);
        robot.chassis.turn_to_heading(180_deg, 1_s);
        robot.match_load_ramp.retract();
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
        robot.match_load_ramp.extend();
        auto pose = robot.chassis.get_pose();
        robot.chassis.set_pose(pose.x(), pose.y(), 0_deg);
        pros::delay(2000);

        // go to 2nd match load
        robot.bottom_intake.set_idle();
        robot.top_intake.set_idle();
        robot.chassis.drive_straight_relative(24_in, 1_s, 0_V, 4_V);
        robot.chassis.move_voltage(4_V, 4_V);
        pros::delay(750);
        robot.bottom_intake.set_intake();
        pros::delay(1000);
        robot.chassis.stop_motors();
        robot.chassis.move_voltage(-6_V, -6_V);
        pros::delay(1500);
        robot.chassis.stop_motors();
        robot.top_intake.set_intake();
        pros::delay(1500);
        pose = robot.chassis.get_pose();
        robot.chassis.set_pose(pose.x(), pose.y(), 0_deg);
        robot.top_intake.set_idle();
        robot.bottom_intake.set_idle();
        robot.chassis.drive_straight_relative(15_in, 1.5_s);
        robot.match_load_ramp.retract();
        robot.chassis.turn_to_heading(60_deg, 1_s);
        robot.chassis.drive_straight_relative(35_in, 1.5_s);
        robot.chassis.turn_to_heading(90_deg, 2_s);
        robot.match_load_ramp.extend();
        robot.bottom_intake.set_intake();
        //robot.top_intake.set_intake();
        robot.chassis.drive_straight_relative(34_in, 3_s, 0_V, 6_V);
        robot.match_load_ramp.retract();
        robot.bottom_intake.set_idle();
        robot.top_intake.set_idle();
        robot.chassis.turn_to_heading(135_deg, 1_s);
        robot.chassis.drive_straight_relative(32_in, 2_s);
        robot.chassis.turn_to_heading(0_deg, 1_s);
        robot.bottom_intake.set_intake();
        robot.chassis.move_voltage(-6_V, -6_V);
        pros::delay(1250);
        robot.chassis.stop_motors();
        robot.top_intake.set_intake();
        robot.match_load_ramp.extend();
        pros::delay(1000);
        robot.chassis.drive_straight_relative(-20_in, 2_s);
        robot.top_intake.set_idle();
        robot.chassis.move_voltage(5_V, 5_V);
        */
    }
}