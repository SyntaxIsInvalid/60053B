#include "main.h"
#include <Eigen/Dense>
#include "abclib/abclib.hpp"
#include <numeric>
#include <mutex>
#include "abclib/builder/path_builder.hpp"
#include "abclib/builder/path_logger.hpp"
#include "abclib/trajectory/trajectory_logger.hpp"
using namespace abclib;

pros::Controller controller(pros::E_CONTROLLER_MASTER);
hardware::motor_group_config left_config{
    .kS = 0.633391,
    .kV = 0.1594417,
    .kPv = 0.2,
    .kIv = 0.0,
    .kDv = 0.0,
};

hardware::motor_group_config right_config{
    .kS = 0.633391,
    .kV = 0.1594417,
    .kPv = 0.2,
    .kIv = 0.0,
    .kDv = 0.0,
};

hardware::AdvancedMotorGroup leftMotors({-10, 4}, pros::MotorGearset::blue, left_config);
hardware::AdvancedMotorGroup rightMotors({6, -11}, pros::MotorGearset::blue, right_config);

// hardware::Pneumatic match_load_ramp('D');

// subsystems::Intake intake({15}, pros::MotorGearset::blue);

pros::IMU imu(9);
pros::Rotation y_rotation(21);
hardware::TrackingWheel y_tracker(&y_rotation, units::Distance::from_inches(2), units::Distance::from_inches(-0.25));

hardware::ChassisConfig chassis_constant{
    .left = &leftMotors,
    .right = &rightMotors,
    .diameter = units::Distance::from_inches(3.25),
    .track_width = units::Distance::from_inches(14),
    .turn_in_place_kS = 1.278592,
    .turn_in_place_kV = 0.170242,
};

hardware::Sensors sensors(&imu, &y_tracker, nullptr);

control::PIDConstants lateral_pid(
    0.5, // kP - proportional
    0,   // kI - integral
    0    // kD - derivative
);

control::PIDConstants angular_pid(
    5, // kP - proportional
    0, // kI - integral
    0  // kD - derivative
);

hardware::Chassis chassis(chassis_constant, sensors, lateral_pid, angular_pid);

void initialize()
{
    pros::lcd::initialize();
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // match_load_ramp.retract();
    chassis.calibrate();
    pros::Task screen_task([&]()
                           {
         while (1) {
             {
                 std::lock_guard<pros::Mutex> lock(telemetry_mutex);
                 telemetry.battery_voltage = units::Voltage::from_millivolts(
                     pros::battery::get_voltage()
                 );
                 telemetry.battery_capacity_percent = pros::battery::get_capacity();
             }

             // Access through telemetry
             TelemetryData local_telem;
             {
                 std::lock_guard<pros::Mutex> lock(telemetry_mutex);
                 local_telem = telemetry;
             }

             // Line 0: X, Y, and Theta
             pros::lcd::print(0, "X:%.2f Y:%.2f Th:%.1f",
                            local_telem.pose.x(),
                            local_telem.pose.y(),
                            local_telem.pose.theta() * 180.0 / M_PI);

             // Line 1: Linear and Angular velocity
             pros::lcd::print(1, "V:%.2f W:%.2f",
                            local_telem.pose_v.inches_per_sec,
                            local_telem.pose_omega.rad_per_sec);

             // Line 2: Battery
             pros::lcd::print(2, "Battery: %.2fV %.0f%%",
                            local_telem.battery_voltage.volts,
                            local_telem.battery_capacity_percent);

             // Line 3: Cross-track and Along-track errors
             pros::lcd::print(3, "XTE:%.2f ATE:%.2f",
                            local_telem.cross_track_error.inches,
                            local_telem.along_track_error.inches);

             // Line 4: Max tracking errors
             pros::lcd::print(4, "MaxXTE:%.2f MaxATE:%.2f",
                            local_telem.max_cross_track_error.inches,
                            local_telem.max_along_track_error.inches);

             // Line 5: Path status and progress
             pros::lcd::print(5, "Status:%s Prog:%.0f%%",
                            path_status_to_string(local_telem.path_status),
                            local_telem.trajectory_progress * 100.0);

             // Line 6: Settlement status
             pros::lcd::print(6, "Settle:%s Cnt:%d",
                            settlement_reason_to_string(local_telem.settlement_reason),
                            local_telem.settle_count);

             // Line 7: Trajectory timing
             pros::lcd::print(7, "Time:%.2f/%.2fs",
                            local_telem.trajectory_time.seconds,
                            local_telem.trajectory_total_time.seconds);

             pros::delay(100);
         } });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

using namespace abclib::path;
void autonomous() {
    characterization::measure_velocity_pid(
        chassis,
        true,
        "velocity_pid_test",
        60.0,
        9000
    );
    controller.print(0, 0, "done");


    // Create path builder
    /*
    PathBuilder builder(units::Distance::from_inches(14.0));

    Path test_path = builder
        .start(0, 0, 0)  // Start at origin facing 0 radians
        
        .begin_profile("forward1", 
                      units::BodyLinearVelocity(36.0),
                      3.0)
        .straight_forward(24.0)  // go forward 24 inches
        
        .begin_profile("turn1",
                      units::BodyLinearVelocity(24.0),
                      2.0)
        .turn_in_place(M_PI)  // turn 180 degrees (turn gets added to turn1 profile)
        
        .begin_profile("forward2",
                      units::BodyLinearVelocity(36.0),
                      3.0)
        .straight_to(0, 0)  // go back to origin using straight_to
        
        .begin_profile("turn2",
                      units::BodyLinearVelocity(24.0),
                      2.0)
        .turn_in_place(0)  // turn back to 0 radians
        
        .build();
    
    chassis.follow_path(test_path, units::Time::from_seconds(15));
    */
    /*
    // Test 6: Complex path mixing everything
    Path test_complex = builder
        .start(0, 0, 0)
        .begin_profile("approach", 
                      units::BodyLinearVelocity(36.0),
                      3.0)
        .spline_to(24, 12, M_PI/6)
        .straight_forward(20.0)
        .begin_profile("turn1",
                      units::BodyLinearVelocity(24.0),
                      2.0)
        .turn_in_place(M_PI/2)
        .begin_profile("pickup", 
                      units::BodyLinearVelocity(12.0),
                      1.0)
        .spline_to(48, 48, M_PI, {{20.0, 20.0, 1.0, 1.0, 0.0, 0.0}})
        .break_continuity()
        .begin_profile("return", 
                      units::BodyLinearVelocity(36.0),
                      3.0)
        .spline_to(24, 24, -M_PI/4)
        .begin_profile("turn2",
                      units::BodyLinearVelocity(20.0),
                      2.5)
        .turn_in_place(-3.0 * M_PI / 4.0)
        .begin_profile("final_approach", 
                    units::BodyLinearVelocity(36.0),
                    3.0)
        .straight_to(0, 0)
        .build();

    
    PathLogger::log_path(test_complex, "test_complex_path");
    trajectory::TrajectoryLogger::log_path_trajectories(test_complex, "test_complex_trajectories");
    pros::lcd::print(0, "All path tests logged!");
    */
}

void opcontrol()
{
    while (1)
    {
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        chassis.drive(throttle, turn, 1, .65);
        /*
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake.set_intake();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intake.set_outtake();
        }
        else
        {
            intake.set_idle();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        {
            match_load_ramp.toggle();
        }

        */
        pros::delay(20);
    }
}