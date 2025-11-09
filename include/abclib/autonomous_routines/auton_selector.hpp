#pragma once
#include "abclib/hardware/chassis.hpp"
#include "abclib/subsystems/intake.hpp"
#include "abclib/hardware/pneumatic.hpp"
#include <map>
#include <functional>
#include "abclib/builder/path_builder.hpp"
#include "abclib/builder/path_logger.hpp"
#include "abclib/trajectory/trajectory_logger.hpp"

namespace abclib::auton {
    enum class AutonRoutine {
        NONE,
        SOLO_AWP_RED,
        SOLO_AWP_BLUE,
        RED_LEFT,
        SKILLS,
        KMS,
        PATH_BUILDER_TEST
    };

    // Subsystems struct to reduce parameter passing
    struct RobotSubsystems {
        hardware::Chassis& chassis;
        subsystems::Intake& top_intake;
        subsystems::Intake& bottom_intake;
        hardware::Pneumatic& match_load_ramp;
        hardware::Pneumatic& intake_lift;
    };

    // Function type for auton routines
    using AutonFunction = std::function<void(RobotSubsystems&)>;

    // Registry of all autons
    inline std::map<AutonRoutine, AutonFunction> auton_registry;

    // Register an auton
    inline void register_auton(AutonRoutine routine, AutonFunction func) {
        auton_registry[routine] = func;
    }

    // Selected auton (change this to select different auton)
    inline AutonRoutine selected_auton = AutonRoutine::SOLO_AWP_RED;

    // Execute the selected auton
    inline void run_selected_auton(RobotSubsystems& robot) {
        if (auton_registry.count(selected_auton) > 0) {
            auton_registry[selected_auton](robot);
        }
        // else: no auton selected or invalid selection
    }
}