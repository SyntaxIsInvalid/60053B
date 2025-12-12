#pragma once
#include "abclib/hardware/chassis.hpp"
#include "abclib/subsystems/intake.hpp"
#include "abclib/hardware/pneumatic.hpp"
#include <map>
#include <vector>
#include <functional>
#include <string>
#include <sstream>
#include "abclib/builder/path_builder.hpp"
#include "abclib/builder/path_logger.hpp"
#include "abclib/trajectory/trajectory_logger.hpp"
#include "abclib/units/units.hpp"
using namespace abclib::units::literals;

namespace abclib::auton {
    enum class AutonRoutine {
        SOLO_AWP_RED,
        SOLO_AWP_BLUE,
        RED_RIGHT,
        RED_LEFT,
        BLUE_RIGHT,
        BLUE_LEFT,
        SKILLS,
        NONE,
        TEST_BOT_AUTON,
        PATH_BUILDER_TEST
    };

    enum class AutonCategory {
        RED,
        BLUE,
        SKILLS,
        TEST
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

    // Auton info structure
    struct AutonInfo {
        AutonFunction function;
        AutonCategory category;
    };

    // Registry of all autons with their info
    inline std::map<AutonRoutine, AutonInfo> auton_registry;

    // Track insertion order per category
    inline std::map<AutonCategory, std::vector<AutonRoutine>> category_order;

    // Register an auton with category
    inline void register_auton(AutonRoutine routine, AutonFunction func, AutonCategory category) {
        auton_registry[routine] = AutonInfo{func, category};
        category_order[category].push_back(routine);
    }

    // Get all autons for a category in insertion order
    inline std::vector<AutonRoutine> get_autons_for_category(AutonCategory category) {
        if (category_order.count(category) > 0) {
            return category_order[category];
        }
        return std::vector<AutonRoutine>(); // Empty vector if category not found
    }

    // Convert enum to display name (SOLO_AWP_RED -> "SOLO AWP RED")
    inline std::string get_display_name(AutonRoutine routine) {
        // Map of enum values to their string names
        static const std::map<AutonRoutine, std::string> enum_names = {
            {AutonRoutine::SOLO_AWP_RED, "SOLO_AWP_RED"},
            {AutonRoutine::SOLO_AWP_BLUE, "SOLO_AWP_BLUE"},
            {AutonRoutine::RED_RIGHT, "RED_RIGHT"},
            {AutonRoutine::RED_LEFT, "RED_LEFT"},
            {AutonRoutine::BLUE_RIGHT, "BLUE_RIGHT"},
            {AutonRoutine::BLUE_LEFT, "BLUE_LEFT"},
            {AutonRoutine::SKILLS, "SKILLS"},
            {AutonRoutine::NONE, "NONE"},
            {AutonRoutine::TEST_BOT_AUTON, "TEST_BOT_AUTON"},
            {AutonRoutine::PATH_BUILDER_TEST, "PATH_BUILDER_TEST"}
        };

        auto it = enum_names.find(routine);
        if (it == enum_names.end()) {
            return "UNKNOWN";
        }

        std::string name = it->second;
        // Replace underscores with spaces
        for (size_t i = 0; i < name.length(); i++) {
            if (name[i] == '_') {
                name[i] = ' ';
            }
        }
        return name;
    }

    // Selected auton (change this to select different auton)
    inline AutonRoutine selected_auton = AutonRoutine::SOLO_AWP_RED;

    // Execute the selected auton
    inline void run_selected_auton(RobotSubsystems& robot) {
        if (auton_registry.count(selected_auton) > 0) {
            auton_registry[selected_auton].function(robot);
        }
        // else: no auton selected or invalid selection
    }
}