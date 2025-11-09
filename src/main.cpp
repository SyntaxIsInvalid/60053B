#include "main.h"
#include <Eigen/Dense>
#include "abclib/abclib.hpp"
#include <numeric>
#include <mutex>
#include "abclib/builder/path_builder.hpp"
#include "abclib/builder/path_logger.hpp"
#include "abclib/trajectory/trajectory_logger.hpp"
#include "liblvgl/lvgl.h"
#include "abclib/autonomous_routines/autons.hpp"
#define TELEMETRY_LEVEL_NONE 0
#define TELEMETRY_LEVEL_MINIMAL 1
#define TELEMETRY_LEVEL_FULL 2

#define TELEMETRY_LEVEL TELEMETRY_LEVEL_MINIMAL 

#define SHOW_TELEOP_IMAGE false
#define SHOW_AUTON_IMAGE false

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
static lv_obj_t *teleop_image = nullptr;

void initialize()
{
    pros::lcd::initialize();
    chassis.calibrate();
    teleop_image = lv_image_create(lv_screen_active());
    // lv_image_set_src(teleop_image, "S:/deft.bin");
    // lv_obj_align(teleop_image, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_add_flag(teleop_image, LV_OBJ_FLAG_HIDDEN); // Hide it initially
    // leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
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
#if TELEMETRY_LEVEL > TELEMETRY_LEVEL_NONE
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
#if TELEMETRY_LEVEL >= TELEMETRY_LEVEL_MINIMAL
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
             pros::lcd::print(2, "%.2fV %.0f%% [%s%.2fx]",
               local_telem.battery_voltage.volts,
               local_telem.battery_capacity_percent,
               local_telem.voltage_compensation_active ? "C" : "-",
               local_telem.voltage_compensation_scale);
#endif
#if TELEMETRY_LEVEL >= TELEMETRY_LEVEL_FULL
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
#endif
             pros::delay(100);
         } });
#endif
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
#if SHOW_AUTON_IMAGE && (TELEMETRY_LEVEL == TELEMETRY_LEVEL_NONE)
    // Only show image if: image display is enabled AND telemetry is off
    if (teleop_image)
    {
        lv_obj_remove_flag(teleop_image, LV_OBJ_FLAG_HIDDEN);
    }
#endif

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
        intake_lift
    };
    run_selected_auton(robot);

    controller.print(0, 0, "done");
}

void opcontrol()
{
#if SHOW_TELEOP_IMAGE && (TELEMETRY_LEVEL == TELEMETRY_LEVEL_NONE)
    // Only show image if: image display is enabled AND telemetry is off
    if (teleop_image) {
        lv_obj_remove_flag(teleop_image, LV_OBJ_FLAG_HIDDEN);
    }
#endif
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