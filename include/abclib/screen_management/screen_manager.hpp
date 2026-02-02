#pragma once
#include "liblvgl/lvgl.h"
#include "abclib/telemetry/telemetry.hpp"
#include <vector>
#include "abclib/autonomous_routines/auton_selector.hpp"

namespace abclib::ui
{
    enum class DefaultScreen
    {
        OVERVIEW = 0,
        PID = 1,
        TRAJECTORY = 2,
        PERFORMANCE = 3,
        BLENDED = 4,
        CONFIG = 5
    };

    class ScreenManager
    {
    private:
        // Main telemetry tabview and tabs
        lv_obj_t *tabview;
        lv_obj_t *tab_overview;
        lv_obj_t *tab_pid;
        lv_obj_t *tab_trajectory;
        lv_obj_t *tab_performance;
        lv_obj_t *tab_config;

        // Autonomous screen tabview and tabs
        lv_obj_t *auton_tabview;
        lv_obj_t *auton_tab_red;
        lv_obj_t *auton_tab_blue;
        lv_obj_t *auton_tab_skills;
        lv_obj_t *auton_tab_test;
        lv_obj_t *auton_tab_telemetry;
        lv_obj_t *auton_tab_image;
        lv_obj_t *tab_blended_debug;                                         // NEW
        std::vector<lv_obj_t *> blended_debug_labels;                        // NEW
        void create_blended_debug_tab();                                     // NEW
        void update_blended_debug_tab(const telemetry::TelemetryData &data); // NEW

        // Labels for each tab (created once, updated repeatedly)
        std::vector<lv_obj_t *> overview_labels;
        std::vector<lv_obj_t *> pid_labels;
        std::vector<lv_obj_t *> trajectory_labels;
        std::vector<lv_obj_t *> performance_labels;
        std::vector<lv_obj_t *> config_labels;

        // Full-screen image object (container)
        lv_obj_t *image_obj;

        // Navigation elements
        lv_obj_t *nav_bar;              // Container for navigation bar
        lv_obj_t *btn_prev;             // Left arrow button
        lv_obj_t *btn_next;             // Right arrow button
        lv_obj_t *btn_auton;            // Auton selector button
        lv_obj_t *btn_image;            // Image button
        lv_obj_t *label_current_screen; // Shows current screen name

        int current_screen_index = 0;        // Track which screen we're on (0-4)
        bool is_auton_screen_active = false; // Track if we're on autonomous screen
        bool is_navigating = false;

        // Tab creation helpers - Telemetry screen
        void create_overview_tab();
        void create_pid_tab();
        void create_trajectory_tab();
        void create_performance_tab();
        void create_config_tab();

        // Tab creation helpers - Autonomous screen
        void create_autonomous_screen();
        void create_auton_red_tab();
        void create_auton_blue_tab();
        void create_auton_skills_tab();
        void create_auton_test_tab();
        void create_auton_telemetry_tab();
        void create_auton_image_tab();

        // Auto-population helpers
        void populate_dropdown(lv_obj_t *dropdown, abclib::auton::AutonCategory category);
        void setup_dropdown_callback(lv_obj_t *dropdown, abclib::auton::AutonCategory category);

        // Navigation helpers
        void create_navigation_bar();
        void navigate_prev();
        void navigate_next();
        void update_current_screen_label();
        void update_navigation_buttons();

        // Screen switching helpers
        void show_telemetry_screen();
        void show_autonomous_screen();

        // Update helpers
        void update_overview_tab(const telemetry::TelemetryData &data);
        void update_pid_tab(const telemetry::TelemetryData &data);
        void update_trajectory_tab(const telemetry::TelemetryData &data);
        void update_performance_tab(const telemetry::TelemetryData &data);
        void update_config_tab(const telemetry::TelemetryData &data);

        // Image display helpers
        void create_fullscreen_image();
        void show_fullscreen_image();
        void hide_fullscreen_image();

        lv_obj_t *calibration_screen;
        lv_obj_t *calibration_bar;
        lv_obj_t *calibration_label;

        void create_calibration_screen();

        // Add to private section
        lv_obj_t *red_dropdown = nullptr;
        lv_obj_t *blue_dropdown = nullptr;
        lv_obj_t *skills_dropdown = nullptr;
        lv_obj_t *test_dropdown = nullptr;

        lv_obj_t *confirm_btn_red = nullptr;
        lv_obj_t *confirm_btn_blue = nullptr;
        lv_obj_t *confirm_btn_skills = nullptr;
        lv_obj_t *confirm_btn_test = nullptr;

        abclib::auton::AutonRoutine temp_selection = abclib::auton::AutonRoutine::NONE;

        // Confirm button tracking and colors
        lv_obj_t *currently_highlighted_btn = nullptr;
        static constexpr uint32_t CONFIRM_COLOR_HIGHLIGHTED = 0x00FF00; // Bright green
        static constexpr uint32_t CONFIRM_COLOR_NORMAL = 0x808080;      // Gray

        void confirm_auton_selection();
        const char *wall_to_string(field::FieldMap::Wall wall);

    public:
        ScreenManager() = default;

        // Initialize all tabs and UI elements
        void initialize(DefaultScreen default_screen = DefaultScreen::OVERVIEW);

        // Update telemetry display (called from loop)
        void update_telemetry(const telemetry::TelemetryData &data);
        void show_calibration_screen();
        void update_calibration_progress(int percentage, const char *status);
        void hide_calibration_screen();
    };

} // namespace abclib