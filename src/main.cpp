#include "main.h"
#include <Eigen/Dense>
#include "abclib/abclib.hpp"
#include <numeric>
#include <mutex>
#include "abclib/builder/path_builder.hpp"
#include "abclib/builder/path_logger.hpp"

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
    chassis.calibrate();/*
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
         } });*/
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

void test_path_builder()
{
    using namespace abclib::path;
    
    PathBuilder builder;
    
    // Calculate the heading needed for the straight segment
    double straight_heading = std::atan2(24 - 12, 48 - 24);  // ≈ 0.4636 rad ≈ 26.57°
    
    Path path = builder
        .start(0, 0, 0)
        
        .begin_profile("approach", units::BodyLinearVelocity(40.0), 30.0)
        .spline_to(24, 12, straight_heading)  // End spline at correct heading
        .straight_to(48, 24)  // Now this works - heading matches!
        
        .begin_profile("precise", units::BodyLinearVelocity(20.0), 15.0)
        .spline_to(60, 36, M_PI / 2)
        
        .build();
    
    PathLogger::log_path(path, "test_path", 2.0);
    
    pros::lcd::print(0, "Path logged!");
    pros::lcd::print(1, "Groups: %d", path.num_groups());
    pros::lcd::print(2, "Length: %.1f in", path.get_total_arc_length());
}


void autonomous()
{
    test_path_builder();
}

void opcontrol()
{
    while (1)
    {
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        chassis.drive(throttle, turn, 1, .65);
        pros::delay(20);
    }
}