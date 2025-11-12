#pragma once

// configs 
#include "abclib/configs/robot_selection.hpp"
#if defined(ROBOT_TEST_DRIVE)
    #include "abclib/configs/test_robot.hpp"
#elif defined(ROBOT_COMPETITION)
    #include "abclib/configs/competition_robot.hpp"
#else
    #error "No robot configuration selected!"
#endif

// units
#include "abclib/units/units.hpp"

// control 
#include "abclib/control/pid.hpp"

// hardware abstractions
#include "abclib/hardware/advanced_motor.hpp"
#include "abclib/hardware/motor_group.hpp"
#include "abclib/hardware/tracking_wheel.hpp"
#include "abclib/hardware/chassis.hpp"
#include "abclib/hardware/motor_tracking_wheel.hpp"
#include "abclib/hardware/tracking_wheel_interface.hpp"

// Pneumatic - real or dummy based on robot config
#if HAS_PNEUMATICS
    #include "abclib/hardware/pneumatic.hpp"
#else
    #include "abclib/hardware/dummy_pneumatic.hpp"
#endif

// state estimation/localization
//#include "abclib/estimation/odometry.hpp"

// math
#include "abclib/math/angles.hpp"
#include "abclib/math/GL15.hpp"
#include "abclib/math/lerp.hpp"
#include "abclib/math/coordinate_frames.hpp"

// path segments 
#include "abclib/path/eta3_segment.hpp"
#include "abclib/path/path_segment_interface.hpp"
#include "abclib/path/straight_segment.hpp"
#include "abclib/path/turn_in_place_segment.hpp"

// motion profiles
#include "abclib/profiling/trapezoidal.hpp"

// telemetry
#include "abclib/telemetry/telemetry.hpp"
#include "abclib/telemetry/logger.hpp"

// kinematic models
#include "abclib/kinematics/differential_drive.hpp"

// trajectory modules
#include "abclib/trajectory/trajectory.hpp"
#include "abclib/trajectory/path_follower.hpp"
#include "abclib/trajectory/trajectory_logger.hpp"

// characterization
#include "abclib/characterization/motor_characterization.hpp"

// builder
#include "abclib/builder/path_builder.hpp"
#include "abclib/builder/path_logger.hpp"

// screen manager
#include "abclib/screen_management/screen_manager.hpp"

// subsystems - real or dummy based on robot config
#if HAS_INTAKE
    #include "abclib/subsystems/intake.hpp"
#else
    #include "abclib/subsystems/dummy_intake.hpp"
#endif