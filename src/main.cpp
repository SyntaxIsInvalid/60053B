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

hardware::ChassisConfig chassis_config{
    .left = &leftMotors,
    .right = &rightMotors,
    .diameter = robot_config::WHEEL_DIAMETER,
    .track_width = robot_config::TRACK_WIDTH,
    .field_config = robot_config::get_estimator_config().field_config,
    .controllers = robot_config::get_controller_config()};

hardware::Chassis chassis(chassis_config, sensors);

// Pneumatics - only for competition robot
#if HAS_PNEUMATICS
hardware::Pneumatic match_load_ramp(robot_config::MATCH_LOAD_RAMP_PORT);
hardware::Pneumatic hood(robot_config::HOOD_PORT);
hardware::Pneumatic wing(robot_config::WING_PORT);
hardware::Pneumatic mid_goal_retract(robot_config::MID_GOAL_RETRACT_PORT);
#else
hardware::DummyPneumatic match_load_ramp;
hardware::DummyPneumatic hood;
hardware::DummyPneumatic wing;
hardware::DummyPneumatic mid_goal_retract;
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
ui::ScreenManager screen_manager;

void initialize()
{
    using namespace abclib::auton;
    using namespace abclib::ui;
    register_auton(AutonRoutine::SOLO_AWP_RED, solo_awp_red, AutonCategory::RED, field::Alliance::RED);
    register_auton(AutonRoutine::RED_LEFT, red_left, AutonCategory::RED, field::Alliance::RED);
    register_auton(AutonRoutine::PATH_BUILDER_TEST, path_builder_test, AutonCategory::TEST, field::Alliance::RED);
    register_auton(AutonRoutine::TEST_BOT_AUTON, test_bot_auton, AutonCategory::TEST, field::Alliance::RED);
    register_auton(AutonRoutine::NONE, none, AutonCategory::TEST, field::Alliance::RED);
    selected_auton = AutonRoutine::TEST_BOT_AUTON;
    lv_init();
    pros::lcd::initialize();
    screen_manager.initialize(DefaultScreen::BLENDED);

    screen_manager.show_calibration_screen();
    chassis.calibrate([](int progress, const char *status)
                      { screen_manager.update_calibration_progress(progress, status); });
    pros::delay(200);
    chassis.set_alliance(field::Alliance::RED);
    chassis.set_pose(0_in, 5_in, 0_deg);
    screen_manager.hide_calibration_screen();
#if HAS_PNEUMATICS
    match_load_ramp.retract();
    hood.retract();
    wing.retract();
#endif
    pros::Task screen_task([&]()
                           {
    while (1) {
        // Update telemetry data in write buffer
        auto& write_buf = abclib::telemetry::g_telemetry.get_write_buffer();
        
        // Battery info
        write_buf.battery_voltage = units::Voltage::from_millivolts(
            pros::battery::get_voltage()
        );
        write_buf.battery_capacity_percent = pros::battery::get_capacity();
        
        // Get both poses from chassis
        write_buf.pose_corner = chassis.get_pose_alliance_corner();
        
        // Get current alliance
        write_buf.current_alliance = chassis.get_alliance();
        write_buf.data_valid = true;

        abclib::telemetry::g_telemetry.swap();
        
        // Update screen with read buffer
        const abclib::telemetry::TelemetryData& data = 
            abclib::telemetry::g_telemetry.get_read_buffer();
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
        hood,
        wing,
        mid_goal_retract};
    run_selected_auton(robot);
    // sysid::measure_ks_kv(leftMotors, rightMotors, true, "ks_kv_comp_forward", 5.5, 0.25, 300);
    // sysid::measure_ks_kv_turn(leftMotors,rightMotors, true, "ks_kv_ccw_comp.csv", 11, 0.25, 500);
    // sysid::measure_velocity_pid(chassis, true, "/usd/vel_200rpm.csv", 200.0, 4500);
    // controller.print(0, 0, "done");
}

void opcontrol()
{
    while (1)
    {
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        chassis.drive(throttle, turn, 1, .65);
        /*
        estimation::Pose robot_pose(0_in, 0_in, 0_deg, 0_ips, 0_rad_per_sec);

        field::FieldMap field_map(robot_config::get_estimator_config().field_config);

        // Test sensor: forward=5, lateral=6 (should be "right" side)
        math::SE2 sensor_pose = field_map.compute_sensor_global_pose(
            robot_pose,
            5_in, // forward
            6_in, // lateral (supposedly RIGHT+)
            0_deg // pointing forward
        );

        // When robot faces EAST:
        // - 5" forward should give X=5 (east)
        // - 6" right should give Y=-6 (south) ... OR Y=+6 (north)?
        controller.print(0, 0, "X:%.1f Y:%.1f",
                         sensor_pose.x(), sensor_pose.y());
        pros::delay(3000);

        // Now test facing NORTH (90°)
        robot_pose.set_theta(90_deg);
        sensor_pose = field_map.compute_sensor_global_pose(
            robot_pose, 5_in, 6_in, 0_deg);

        // When robot faces NORTH:
        // - 5" forward should give Y=5 (north)
        // - 6" right should give X=6 (east) ... OR X=-6 (west)?
        controller.print(0, 0, "X:%.1f Y:%.1f",
                         sensor_pose.x(), sensor_pose.y());
        */
#if HAS_INTAKE && HAS_PNEUMATICS
        // intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            top_intake.set_voltage(4_V);
            bottom_intake.set_intake();
            hood.extend();
        }
        // outake
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            top_intake.set_outtake();
            bottom_intake.set_outtake();
        }
        // score mid
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            mid_goal_retract.extend();
            top_intake.set_outtake();
            bottom_intake.set_intake();
            // score long
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            hood.retract();
            top_intake.set_intake();
            bottom_intake.set_intake();
        }
        else
        {
            mid_goal_retract.retract();
            top_intake.set_voltage(3_V);
            bottom_intake.set_idle();
        }
#endif

#if HAS_PNEUMATICS
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            match_load_ramp.toggle();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            wing.toggle();
        }
#endif
        pros::delay(20);
    }
}