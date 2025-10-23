#pragma once

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

// state estimation/localization
#include "abclib/estimation/odometry.hpp"

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

// characterization
#include "abclib/characterization/motor_characterization.hpp"


