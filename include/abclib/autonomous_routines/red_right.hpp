#pragma once
#include "auton_selector.hpp"

namespace abclib::auton {
    inline void red_right(RobotSubsystems& robot) {
         robot.chassis.set_pose(
            units::Distance::from_inches(0),
            units::Distance::from_inches(0),
            units::Degrees(0));
            robot.wing.retract();
            robot.bottom_intake.set_intake();
        robot.chassis.drive_straight_relative(15_in, 3_s);
        robot.chassis.turn_to_heading(-35_deg, 1_s);
        robot.chassis.drive_straight_relative(10_in, 5_s, 0_V, 3_V);
        robot.chassis.turn_to_heading(-135_deg, 2_s);
        robot.chassis.drive_straight_relative(37_in, 5_s);
        robot.match_load_ramp.extend();
        robot.chassis.turn_to_heading(180_deg, 2_s);
        robot.chassis.drive_straight_relative(12_in, 1_s);
        //robot.chassis.move_voltage(2_V, 2_V);
        robot.chassis.drive_straight_relative(-40_in, 2_s);
        robot.top_intake.set_intake();
        pros::delay(2000);
        robot.chassis.drive_straight_relative(12_in, 5_s);
        robot.chassis.turn_to_heading(120_deg, 1_s);
        robot.chassis.drive_straight_relative(-11_in, 2_s);
        robot.match_load_ramp.retract();
        robot.chassis.turn_to_heading(180_deg, 2_s);
        robot.chassis.drive_straight_relative(-22_in, 5_s);
    }
}