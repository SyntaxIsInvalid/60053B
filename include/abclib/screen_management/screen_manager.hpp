#pragma once
#include "liblvgl/lvgl.h"
#include "abclib/telemetry/telemetry.hpp"
#include <vector>

namespace abclib::ui
{
    class ScreenManager
    {
    private:
        // Tabview and tabs
        lv_obj_t *tabview;
        lv_obj_t *tab_overview;
        lv_obj_t *tab_pid;
        lv_obj_t *tab_trajectory;
        lv_obj_t *tab_performance;
        lv_obj_t *tab_config;

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

        int current_screen_index = 0;   // Track which screen we're on (0-4)

        // Tab creation helpers
        void create_overview_tab();
        void create_pid_tab();
        void create_trajectory_tab();
        void create_performance_tab();
        void create_config_tab();
        
        // Navigation helpers
        void create_navigation_bar();
        void navigate_prev();
        void navigate_next();
        void update_current_screen_label();
        void update_navigation_buttons();

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

    public:
        ScreenManager() = default;

        // Initialize all tabs and UI elements
        void initialize();

        // Update telemetry display (called from loop)
        void update_telemetry(const telemetry::TelemetryData &data);
    };

} // namespace abclib