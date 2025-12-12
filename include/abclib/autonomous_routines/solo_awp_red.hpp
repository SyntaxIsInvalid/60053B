#pragma once
#include "auton_selector.hpp"

namespace abclib::auton
{
    inline void solo_awp_red(RobotSubsystems &robot)
    {
        // Set starting position
        robot.chassis.set_pose(
            units::Distance::from_inches(0),
            units::Distance::from_inches(0),
            units::Degrees(0));
        
        robot.chassis.move_to_pose_profiled(12_in, 6_in, 45_deg, 12_ips, 40_ips2, 10_s);
        /*
        robot.chassis.set_pose(
            units::Distance::from_inches(0),
            units::Distance::from_inches(0),
            units::Degrees(0));

        robot.chassis.drive_straight_relative(units::Distance(25), units::Time::from_seconds(1.2));
        robot.match_load_ramp.extend();
        pros::delay(500);
        robot.chassis.turn_to_heading_profiled_pid(units::Degrees(-95), 3.14, 6.28, units::Time::from_seconds(1.5));
        robot.bottom_intake.intake_for(units::Time::from_seconds(5));
        robot.chassis.drive_straight_relative(units::Distance(17), units::Time::from_seconds(1), units::Voltage::from_volts(0), units::Voltage::from_volts(4.5));
        robot.top_intake.intake_for(units::Time::from_seconds(0.6));
        pros::delay(500);
        robot.chassis.drive_straight_relative(units::Distance(-8), units::Time::from_seconds(.7));
        robot.match_load_ramp.retract();
        robot.chassis.turn_to_heading_profiled_pid(units::Degrees(95), 3.14, 6.28, units::Time::from_seconds(1.3));
        pros::delay(250);
        robot.intake_lift.extend();
        robot.chassis.drive_straight_relative(units::Distance(22), units::Time::from_seconds(.7));
        robot.top_intake.intake_for(units::Time::from_seconds(2));
        pros::delay(2000);
        robot.chassis.drive_straight_relative(units::Distance(-20), units::Time::from_seconds(.7));
        robot.intake_lift.retract();
        robot.chassis.turn_relative(units::Degrees(45), units::Time::from_seconds(1));
        robot.chassis.drive_straight_relative(units::Distance(20), units::Time::from_seconds(.7));
        robot.bottom_intake.intake_for_voltage(units::Voltage::from_volts(12),units::Time::from_seconds(2));
        robot.chassis.drive_straight_relative(units::Distance(18), units::Time::from_seconds(1), units::Voltage::from_volts(0), units::Voltage::from_volts(5));
        pros::delay(1000);
        robot.top_intake.outtake_for_voltage(units::Voltage::from_volts(12), units::Time::from_seconds(1));
        robot.bottom_intake.outtake_for_voltage(units::Voltage::from_volts(12), units::Time::from_seconds(1));
        */
    }
}