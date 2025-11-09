#pragma once
#include "auton_selector.hpp"

namespace abclib::auton {
    inline void red_left(RobotSubsystems& robot) {
        // Set starting position
        robot.chassis.set_pose(
            units::Distance::from_inches(0),
            units::Distance::from_inches(0),
            units::Degrees(0));
        robot.match_load_ramp.extend();
        robot.chassis.drive_straight_relative(units::Distance(30), units::Time::from_seconds(1.2));
        robot.chassis.turn_to_heading(units::Degrees(90), units::Time::from_seconds(.5));
        robot.bottom_intake.intake_for(units::Time::from_seconds(5));
        robot.chassis.drive_straight_relative(units::Distance(17), units::Time::from_seconds(1), units::Voltage::from_volts(0), units::Voltage::from_volts(4.5));
        robot.top_intake.intake_for(units::Time::from_seconds(0.6));
        pros::delay(500);
        robot.chassis.drive_straight_relative(units::Distance(-8), units::Time::from_seconds(.7));
        robot.match_load_ramp.retract();
        robot.chassis.turn_to_heading(units::Degrees(-90), units::Time::from_seconds(1));
        pros::delay(250);
        robot.intake_lift.extend();
        robot.chassis.drive_straight_relative(units::Distance(22), units::Time::from_seconds(.7));
        robot.top_intake.intake_for(units::Time::from_seconds(2));
    }
}