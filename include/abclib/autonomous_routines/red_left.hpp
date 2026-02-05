#pragma once
#include "auton_selector.hpp"

namespace abclib::auton {
    inline void red_left(RobotSubsystems& robot) {
                 robot.chassis.set_pose(
            units::Distance::from_inches(0),
            units::Distance::from_inches(0),
            units::Degrees(0));
            robot.wing.retract();
            robot.hood.extend();
            robot.bottom_intake.set_voltage(8_V);
        robot.chassis.drive_straight_relative(15_in, 3_s);
        robot.chassis.turn_to_heading(35_deg, 1_s);
        robot.chassis.drive_straight_relative(12_in, 5_s, 0_V, 3_V);
        robot.chassis.turn_to_heading(135_deg, 2_s);
        robot.bottom_intake.set_idle();
        robot.chassis.drive_straight_relative(-17_in, 1_s);
        robot.chassis.turn_to_heading(135_deg, 0.3_s);
        robot.top_intake.set_idle();
        robot.mid_goal.extend();
        robot.bottom_intake.intake_for_voltage(-12_V, .01_s);
        robot.top_intake.intake_for_voltage(-12_V, 1_s);
        robot.bottom_intake.set_intake();
        pros::delay(1000);
        robot.chassis.drive_straight_relative(46_in, 2_s);
        robot.mid_goal.retract();
                robot.match_load_ramp.extend();
        robot.chassis.turn_to_heading(180_deg, 1_s);
        robot.chassis.drive_straight_relative(17_in, 1.5_s);
        robot.hood.retract();
        robot.bottom_intake.set_intake();
        robot.chassis.drive_straight_relative(-28_in, 1_s);
        pros::delay(500);
        robot.top_intake.set_intake();
        pros::delay(500);
        robot.chassis.drive_straight_relative(12_in, 5_s);
        robot.chassis.turn_to_heading(120_deg, 1_s);
        robot.chassis.drive_straight_relative(-11_in, 2_s);
        robot.match_load_ramp.retract();
        robot.chassis.turn_to_heading(180_deg, 2_s);
        robot.chassis.drive_straight_relative(-22_in, 5_s);
    }
}