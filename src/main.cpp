#include "main.h"
#include <Eigen/Dense>
#include "abclib/abclib.hpp"
#include <numeric>
#include <mutex>
#include "liblvgl/lvgl.h"
#include "abclib/autonomous_routines/autons.hpp"

using namespace abclib;

#ifdef ROBOT_TEST_DRIVE
#include "abclib/configs/test_robot.hpp"
#elif defined(ROBOT_COMPETITION)
#include "abclib/configs/competition_robot.hpp"
#else
#error "No robot configuration selected!"
#endif

pros::Controller controller(pros::E_CONTROLLER_MASTER);

using namespace abclib::robot_config;

hardware::AdvancedMotorGroup leftMotors(
    robot_config::LEFT_MOTOR_PORTS,
    pros::MotorGearset::blue,
    robot_config::get_left_motor_config());

hardware::AdvancedMotorGroup rightMotors(
    robot_config::RIGHT_MOTOR_PORTS,
    pros::MotorGearset::blue,
    robot_config::get_right_motor_config());

pros::IMU imu(robot_config::IMU_PORT);

#if USE_ROTATION_TRACKER
// Competition robot - uses rotation sensor
pros::Rotation y_rotation(robot_config::Y_ROTATION_PORT);
hardware::TrackingWheel y_tracker(
    &y_rotation,
    robot_config::Y_TRACKER_WHEEL_DIAMETER,
    robot_config::Y_TRACKER_OFFSET);
hardware::Sensors sensors(&imu, &y_tracker, nullptr);
#else
// Test drive - uses motor tracking
hardware::MotorTrackingWheel y_tracker(
    &leftMotors,
    robot_config::WHEEL_DIAMETER,
    robot_config::Y_TRACKER_OFFSET);
hardware::Sensors sensors(&imu, &y_tracker, nullptr);
#endif

hardware::ChassisConfig chassis_constant{
    .left = &leftMotors,
    .right = &rightMotors,
    .diameter = robot_config::WHEEL_DIAMETER,
    .track_width = robot_config::TRACK_WIDTH,
    .turn_in_place_kS = robot_config::TURN_IN_PLACE_KS,
    .turn_in_place_kV = robot_config::TURN_IN_PLACE_KV,
    .turn_in_place_kA = robot_config::TURN_IN_PLACE_KA,
    .profiled_turn_pid_constants = robot_config::get_profiled_turn_pid(),
    .profiled_lateral_pid_constants = robot_config::get_profiled_lateral_pid(),
    .lateral_kS = robot_config::LATERAL_KS,
    .lateral_kV = robot_config::LATERAL_KV,
    .lateral_kA = robot_config::LATERAL_KA,
};

hardware::Chassis chassis(
    chassis_constant,
    sensors,
    robot_config::get_lateral_pid(),
    robot_config::get_angular_pid());

// Pneumatics - only for competition robot
#if HAS_PNEUMATICS
hardware::Pneumatic match_load_ramp(robot_config::MATCH_LOAD_RAMP_PORT);
hardware::Pneumatic intake_lift(robot_config::INTAKE_LIFT_PORT);
#else
hardware::DummyPneumatic match_load_ramp;
hardware::DummyPneumatic intake_lift;
#endif

#if HAS_INTAKE
subsystems::Intake top_intake(
    robot_config::TOP_INTAKE_PORTS,
    pros::MotorGearset::blue,
    hardware::motor_group_config{},
    robot_config::TOP_INTAKE_VOLTAGE,
    robot_config::TOP_OUTTAKE_VOLTAGE);

subsystems::Intake bottom_intake(
    robot_config::BOTTOM_INTAKE_PORTS,
    pros::MotorGearset::blue,
    hardware::motor_group_config{},
    robot_config::BOTTOM_INTAKE_VOLTAGE,
    robot_config::BOTTOM_OUTTAKE_VOLTAGE);
#else
subsystems::DummyIntake top_intake;
subsystems::DummyIntake bottom_intake;
#endif
// static lv_obj_t *teleop_image = nullptr;
abclib::ScreenManager screen_manager;

void initialize()
{
    lv_init();

    pros::lcd::initialize();
    chassis.calibrate();
    screen_manager.initialize();
    using namespace abclib::auton;
    register_auton(AutonRoutine::SOLO_AWP_RED, solo_awp_red);
    register_auton(AutonRoutine::PATH_BUILDER_TEST, path_builder_test);
    register_auton(AutonRoutine::RED_LEFT, red_left);
    register_auton(AutonRoutine::NONE, none);
    register_auton(AutonRoutine::TEST_BOT_AUTON, test_bot_auton);
#if HAS_PNEUMATICS
    match_load_ramp.retract();
    intake_lift.retract();
#endif
    pros::Task screen_task([&]()
                           {
        while (1) {
            // Update battery info in the write buffer
            auto& write_buf = abclib::telemetry.get_write_buffer();
            write_buf.battery_voltage = units::Voltage::from_millivolts(
                pros::battery::get_voltage()
            );
            write_buf.battery_capacity_percent = pros::battery::get_capacity();
            abclib::telemetry.swap();
            
            // Update screen with read buffer
            const TelemetryData& data = abclib::telemetry.get_read_buffer();
            screen_manager.update_telemetry(data);
            
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

void autonomous()
{
    chassis.set_pose(
        units::Distance::from_inches(0),
        units::Distance::from_inches(0),
        units::Degrees(0));
    using namespace abclib::auton;

    // Create subsystems struct
    RobotSubsystems robot{
        chassis,
        top_intake,
        bottom_intake,
        match_load_ramp,
        intake_lift};
    run_selected_auton(robot);

    controller.print(0, 0, "done");
}

void opcontrol()
{
    while (1)
    {
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        chassis.drive(throttle, turn, 1, .65);
#if HAS_INTAKE
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            top_intake.set_voltage(units::Voltage::from_volts(3));
            bottom_intake.set_intake();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            top_intake.set_outtake();
            bottom_intake.set_outtake();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            top_intake.set_intake();
            bottom_intake.set_intake();
        }
        else
        {
            top_intake.set_idle();
            bottom_intake.set_idle();
        }
#endif

#if HAS_PNEUMATICS
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            match_load_ramp.toggle();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            intake_lift.toggle();
        }
#endif
        pros::delay(20);
    }
}