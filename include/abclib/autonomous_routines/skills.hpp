#pragma once
#include "auton_selector.hpp"

using namespace abclib::units::literals;

namespace abclib::auton
{
    inline void skills(RobotSubsystems &robot)
    {
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
        robot.chassis.drive_straight_relative(24_in, 1_s, 0_V, 6_V);
        robot.chassis.move_voltage(5_V, 5_V);
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
        // go towards park
        
        robot.chassis.set_pose(80_in, 80_in, 0_deg);
        robot.top_intake.set_idle();
        robot.bottom_intake.set_idle();
        robot.chassis.drive_straight_relative(15_in, 1.5_s);
        robot.match_load_ramp.retract();
        robot.chassis.turn_to_heading(60_deg, 1_s);
        robot.chassis.drive_straight_relative(35_in, 1.5_s);
        robot.chassis.turn_to_heading(90_deg, 2_s);
        robot.match_load_ramp.extend();
        robot.bottom_intake.set_intake();
        robot.chassis.drive_straight_relative(30_in, 2.5_s, 0_V, 7_V);
        robot.chassis.drive_straight_relative(-13_in, .8_s);
        
        // go towards next matchload
        
        robot.wing.extend();
        robot.chassis.set_pose(53_in, 137.75_in, 90_deg);
        robot.match_load_ramp.retract();
        robot.bottom_intake.set_idle();
        robot.top_intake.set_idle();
        robot.chassis.turn_to_heading(120_deg, 1_s);
        
        robot.chassis.drive_straight_relative(29_in, 1.6_s);
        
        robot.chassis.turn_to_heading(0_deg, 1_s);
        robot.bottom_intake.set_intake();
        robot.chassis.drive_straight_relative(-24_in, 1.3_s);
        robot.top_intake.set_intake();
        robot.match_load_ramp.extend();
        pros::delay(750);
        robot.top_intake.set_idle();
       robot.bottom_intake.set_intake();
       robot.match_load_ramp.extend();
        robot.chassis.drive_straight_relative(30_in, 1.2_s, 0_V, 7_V);
        // go towards last matchload
        robot.chassis.drive_straight_relative(-16_in, 1_s);
        robot.chassis.turn_to_heading(-45_deg);
        robot.chassis.drive_straight_relative(-14_in, 1_s);
        robot.chassis.turn_to_heading(0_deg, .7_s);
        robot.chassis.drive_straight_relative(-66_in, 2_s);
        robot.chassis.turn_to_heading(45_deg, 1_s);
        robot.chassis.drive_straight_relative(-13_in, 0.4_s);
        robot.chassis.turn_to_heading(180_deg, 1_s);
        robot.chassis.move_voltage(-8_V, -8_V);
        pros::delay(850);
        robot.bottom_intake.set_intake();
        robot.top_intake.set_intake();
        robot.match_load_ramp.extend();
        pros::delay(1000);
        robot.chassis.drive_straight_relative(26_in, 2_s);
        
       
       robot.chassis.set_pose(0_in,0_in,180_deg);
       robot.chassis.drive_straight_relative(18_in, 1_s);
       robot.chassis.turn_to_heading(-110_deg, 1.5_s);
       robot.bottom_intake.set_intake();
       robot.chassis.drive_straight_relative(48_in);
       
    }
}