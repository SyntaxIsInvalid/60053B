#include "screen_manager.hpp"
#include <cstdio>

namespace abclib::ui {

void ScreenManager::initialize() {
    // Create the main telemetry tabview
    tabview = lv_tabview_create(lv_screen_active());
    
    // Hide the default tab bar (we'll use custom navigation)
    lv_tabview_set_tab_bar_size(tabview, 0);
    
    // Create all telemetry tabs
    tab_overview = lv_tabview_add_tab(tabview, "Overview");
    tab_pid = lv_tabview_add_tab(tabview, "PID");
    tab_trajectory = lv_tabview_add_tab(tabview, "Trajectory");
    tab_performance = lv_tabview_add_tab(tabview, "Performance");
    tab_config = lv_tabview_add_tab(tabview, "Config");
    
    // Setup all telemetry tabs
    create_overview_tab();
    create_pid_tab();
    create_trajectory_tab();
    create_performance_tab();
    create_config_tab();
    
    // Create custom navigation bar
    create_navigation_bar();
    
    // Create full-screen image overlay (not a tab anymore)
    create_fullscreen_image();
    create_calibration_screen();
    // Create autonomous screen (initially hidden)
    create_autonomous_screen();
}

void ScreenManager::create_navigation_bar() {
    // Create container for navigation bar at the top
    nav_bar = lv_obj_create(lv_screen_active());
    lv_obj_set_size(nav_bar, LV_PCT(100), 50);
    lv_obj_align(nav_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_pad_all(nav_bar, 5, 0);
    lv_obj_remove_flag(nav_bar, LV_OBJ_FLAG_SCROLLABLE);
    
    // Left arrow button
    btn_prev = lv_button_create(nav_bar);
    lv_obj_set_size(btn_prev, 40, 40);
    lv_obj_align(btn_prev, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_t* label_prev = lv_label_create(btn_prev);
    lv_label_set_text(label_prev, LV_SYMBOL_LEFT);
    lv_obj_center(label_prev);
    lv_obj_add_event_cb(btn_prev, [](lv_event_t* e) {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->navigate_prev();
    }, LV_EVENT_CLICKED, this);
    
    // Auton selector button (now enabled)
    btn_auton = lv_button_create(nav_bar);
    lv_obj_set_size(btn_auton, 80, 40);
    lv_obj_align(btn_auton, LV_ALIGN_LEFT_MID, 45, 0);
    lv_obj_t* label_auton = lv_label_create(btn_auton);
    lv_label_set_text(label_auton, "Auton");
    lv_obj_center(label_auton);
    lv_obj_add_event_cb(btn_auton, [](lv_event_t* e) {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->show_autonomous_screen();
    }, LV_EVENT_CLICKED, this);
    
    // Current screen name label (center)
    label_current_screen = lv_label_create(nav_bar);
    lv_label_set_text(label_current_screen, "Overview");
    lv_obj_align(label_current_screen, LV_ALIGN_CENTER, 0, 0);
    
    // Image button
    btn_image = lv_button_create(nav_bar);
    lv_obj_set_size(btn_image, 80, 40);
    lv_obj_align(btn_image, LV_ALIGN_RIGHT_MID, -45, 0);
    lv_obj_t* label_image = lv_label_create(btn_image);
    lv_label_set_text(label_image, "Image");
    lv_obj_center(label_image);
    lv_obj_add_event_cb(btn_image, [](lv_event_t* e) {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->show_fullscreen_image();
    }, LV_EVENT_CLICKED, this);
    
    // Right arrow button
    btn_next = lv_button_create(nav_bar);
    lv_obj_set_size(btn_next, 40, 40);
    lv_obj_align(btn_next, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_t* label_next = lv_label_create(btn_next);
    lv_label_set_text(label_next, LV_SYMBOL_RIGHT);
    lv_obj_center(label_next);
    lv_obj_add_event_cb(btn_next, [](lv_event_t* e) {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->navigate_next();
    }, LV_EVENT_CLICKED, this);
    
    // Update button states for initial screen
    update_navigation_buttons();
}

void ScreenManager::create_autonomous_screen() {
    // Create the autonomous tabview (initially hidden)
    auton_tabview = lv_tabview_create(lv_screen_active());
    
    // Position it to cover the entire screen
    lv_obj_set_size(auton_tabview, LV_PCT(100), LV_PCT(100));
    lv_obj_align(auton_tabview, LV_ALIGN_TOP_LEFT, 0, 0);
    
    // Create all 6 tabs
    auton_tab_red = lv_tabview_add_tab(auton_tabview, "Red");
    auton_tab_blue = lv_tabview_add_tab(auton_tabview, "Blue");
    auton_tab_skills = lv_tabview_add_tab(auton_tabview, "Skills");
    auton_tab_test = lv_tabview_add_tab(auton_tabview, "Test");
    auton_tab_telemetry = lv_tabview_add_tab(auton_tabview, "Telemetry");
    auton_tab_image = lv_tabview_add_tab(auton_tabview, "Image");
    
    // Setup all autonomous tabs
    create_auton_red_tab();
    create_auton_blue_tab();
    create_auton_skills_tab();
    create_auton_test_tab();
    create_auton_telemetry_tab();
    create_auton_image_tab();
    
    // Add event callback to detect tab changes
    lv_obj_add_event_cb(auton_tabview, [](lv_event_t* e) {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        lv_obj_t* tabview = (lv_obj_t*)lv_event_get_target(e);
        uint32_t active_tab = lv_tabview_get_tab_active(tabview);
        
        // Tab 4 is Telemetry - switch back to main telemetry screen
        if (active_tab == 4) {
            manager->show_telemetry_screen();
        }
        // Tab 5 is Image - show fullscreen image
        else if (active_tab == 5) {
            manager->show_fullscreen_image();
        }
    }, LV_EVENT_VALUE_CHANGED, this);
    
    // Hide initially
    lv_obj_add_flag(auton_tabview, LV_OBJ_FLAG_HIDDEN);
}

void ScreenManager::create_auton_red_tab() {
    lv_obj_t* label = lv_label_create(auton_tab_red);
    lv_label_set_text(label, "Red Autonomous\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_auton_blue_tab() {
    lv_obj_t* label = lv_label_create(auton_tab_blue);
    lv_label_set_text(label, "Blue Autonomous\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_auton_skills_tab() {
    lv_obj_t* label = lv_label_create(auton_tab_skills);
    lv_label_set_text(label, "Skills Autonomous\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_auton_test_tab() {
    lv_obj_t* label = lv_label_create(auton_tab_test);
    lv_label_set_text(label, "Test Autonomous\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_auton_telemetry_tab() {
    lv_obj_t* label = lv_label_create(auton_tab_telemetry);
    lv_label_set_text(label, "Return to Telemetry");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_auton_image_tab() {
    lv_obj_t* label = lv_label_create(auton_tab_image);
    lv_label_set_text(label, "Show Image");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::show_telemetry_screen() {
    // Hide autonomous screen
    lv_obj_add_flag(auton_tabview, LV_OBJ_FLAG_HIDDEN);
    
    // Show telemetry tabview and nav bar
    lv_obj_remove_flag(tabview, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(nav_bar, LV_OBJ_FLAG_HIDDEN);
    
    is_auton_screen_active = false;
}

void ScreenManager::show_autonomous_screen() {
    // Hide telemetry tabview and nav bar
    lv_obj_add_flag(tabview, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(nav_bar, LV_OBJ_FLAG_HIDDEN);
    
    // Show autonomous screen
    lv_obj_remove_flag(auton_tabview, LV_OBJ_FLAG_HIDDEN);
    
    // Set to first tab (Red)
    lv_tabview_set_active(auton_tabview, 0, LV_ANIM_OFF);
    
    is_auton_screen_active = true;
}

void ScreenManager::navigate_prev() {
    if (is_navigating || current_screen_index <= 0) {
        return;  // Exit early if already navigating or at first screen
    }
    
    is_navigating = true;
    
    current_screen_index--;
    lv_tabview_set_active(tabview, current_screen_index, LV_ANIM_ON);
    update_current_screen_label();
    update_navigation_buttons();
    
    is_navigating = false;
}

void ScreenManager::navigate_next() {
    if (is_navigating || current_screen_index >= 4) {
        return;  // Exit early if already navigating or at last screen
    }
    
    is_navigating = true;
    
    current_screen_index++;
    lv_tabview_set_active(tabview, current_screen_index, LV_ANIM_ON);
    update_current_screen_label();
    update_navigation_buttons();
    
    is_navigating = false;
}


void ScreenManager::update_current_screen_label() {
    const char* screen_names[] = {"Overview", "PID", "Trajectory", "Performance", "Config"};
    lv_label_set_text(label_current_screen, screen_names[current_screen_index]);
}

void ScreenManager::update_navigation_buttons() {
    // Hide/show left arrow
    if (current_screen_index == 0) {
        lv_obj_add_flag(btn_prev, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_remove_flag(btn_prev, LV_OBJ_FLAG_HIDDEN);
    }
    
    // Hide/show right arrow
    if (current_screen_index == 4) {  // Last screen (index 4)
        lv_obj_add_flag(btn_next, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_remove_flag(btn_next, LV_OBJ_FLAG_HIDDEN);
    }
}

void ScreenManager::create_overview_tab() {
    // Create labels for overview data
    for (int i = 0; i < 3; i++) {
        lv_obj_t* label = lv_label_create(tab_overview);
        lv_label_set_text(label, "Loading...");
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 60 + i * 40);  // Offset for nav bar
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(label, 450);
        overview_labels.push_back(label);
    }
}

void ScreenManager::create_pid_tab() {
    lv_obj_t* label = lv_label_create(tab_pid);
    lv_label_set_text(label, "PID Control Data\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_trajectory_tab() {
    lv_obj_t* label = lv_label_create(tab_trajectory);
    lv_label_set_text(label, "Trajectory Tracking Data\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_performance_tab() {
    lv_obj_t* label = lv_label_create(tab_performance);
    lv_label_set_text(label, "Performance Metrics\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_config_tab() {
    lv_obj_t* label = lv_label_create(tab_config);
    lv_label_set_text(label, "Tuning Configuration\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_fullscreen_image() {
    // Create the full-screen image container (initially hidden)
    image_obj = lv_obj_create(lv_screen_active());
    lv_obj_set_size(image_obj, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(image_obj, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(image_obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(image_obj, 0, 0);
    lv_obj_set_style_pad_all(image_obj, 0, 0);
    
    // Check if the image file exists
    FILE* test_file = fopen("/usd/images/deft.bin", "r");
    bool image_exists = (test_file != nullptr);
    if (test_file) {
        fclose(test_file);
    }
    
    if (image_exists) {
        // Image exists - create and display it
        lv_obj_t* img = lv_image_create(image_obj);
        lv_image_set_src(img, "S:/images/deft.bin");
        lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    } else {
        // Image doesn't exist - show error message
        lv_obj_t* error_label = lv_label_create(image_obj);
        lv_label_set_text(error_label, 
            "ERROR:\n"
            "SD Card not found\n"
            "or\n"
            "Image file missing\n\n"
            "Expected:\n"
            "S:/images/deft.bin");
        lv_obj_set_style_text_color(error_label, lv_color_white(), 0);
        lv_obj_set_style_text_align(error_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(error_label, LV_ALIGN_CENTER, 0, 0);
    }
    
    // Make it clickable to close
    lv_obj_add_flag(image_obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(image_obj, [](lv_event_t* e) {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->hide_fullscreen_image();
    }, LV_EVENT_CLICKED, this);
    
    // Hide it initially
    lv_obj_add_flag(image_obj, LV_OBJ_FLAG_HIDDEN);
}

void ScreenManager::show_fullscreen_image() {
    lv_obj_remove_flag(image_obj, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_to_index(image_obj, -1);  // Move to front
}

void ScreenManager::hide_fullscreen_image() {
    lv_obj_add_flag(image_obj, LV_OBJ_FLAG_HIDDEN);
    
    // If we were on the autonomous screen, switch to telemetry tab
    if (is_auton_screen_active) {
        lv_tabview_set_active(auton_tabview, 4, LV_ANIM_OFF);  // Tab 4 is Telemetry
    }
}

void ScreenManager::update_telemetry(const telemetry::TelemetryData& data) {
    update_overview_tab(data);
    // Other tabs will be implemented later
}

void ScreenManager::update_overview_tab(const telemetry::TelemetryData& data) {
    if (overview_labels.size() >= 3) {
        char buf0[64];
        char buf1[64];
        char buf2[64];
        
        // Label 0: Pose (using the new units API)
        snprintf(buf0, sizeof(buf0), "X:%.2f Y:%.2f Th:%.1f",
            data.pose.x_inches(),
            data.pose.y_inches(),
            data.pose.theta_deg());
        lv_label_set_text(overview_labels[0], buf0);
        
        // Label 1: Velocity (using the new units API)
        snprintf(buf1, sizeof(buf1), "V:%.2f W:%.2f",
            data.pose_v_raw.to_ips(),
            data.pose_omega_raw.to_rad_per_sec());
        lv_label_set_text(overview_labels[1], buf1);
        
        // Label 2: Battery (using the new units API)
        snprintf(buf2, sizeof(buf2), "%.2fV %.0f%% [%s%.2fx]",
            data.battery_voltage.to_volts(),
            data.battery_capacity_percent,
            data.voltage_compensation_active ? "C" : "-",
            data.voltage_compensation_scale);
        lv_label_set_text(overview_labels[2], buf2);
    }
}

void ScreenManager::create_calibration_screen() {
    // Create full-screen container
    calibration_screen = lv_obj_create(lv_screen_active());
    lv_obj_set_size(calibration_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(calibration_screen, lv_color_hex(0x1a1a1a), 0);
    lv_obj_remove_flag(calibration_screen, LV_OBJ_FLAG_SCROLLABLE);
    
    // Status label
    calibration_label = lv_label_create(calibration_screen);
    lv_label_set_text(calibration_label, "Calibrating IMU...");
    lv_obj_set_style_text_font(calibration_label, &lv_font_montserrat_24, 0);
    lv_obj_align(calibration_label, LV_ALIGN_CENTER, 0, -40);
    
    // Progress bar
    calibration_bar = lv_bar_create(calibration_screen);
    lv_obj_set_size(calibration_bar, 300, 30);
    lv_obj_align(calibration_bar, LV_ALIGN_CENTER, 0, 20);
    lv_bar_set_value(calibration_bar, 0, LV_ANIM_OFF);
    
    // Hide initially
    lv_obj_add_flag(calibration_screen, LV_OBJ_FLAG_HIDDEN);
}

void ScreenManager::show_calibration_screen() {
    lv_obj_remove_flag(calibration_screen, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_to_index(calibration_screen, -1);  // Move to front
    lv_bar_set_value(calibration_bar, 0, LV_ANIM_OFF);
}

void ScreenManager::update_calibration_progress(int percentage, const char* status) {
    lv_bar_set_value(calibration_bar, percentage, LV_ANIM_ON);
    lv_label_set_text(calibration_label, status);
}

void ScreenManager::hide_calibration_screen() {
    lv_obj_add_flag(calibration_screen, LV_OBJ_FLAG_HIDDEN);
}


// Stub implementations for tabs not yet implemented
void ScreenManager::update_pid_tab(const telemetry::TelemetryData& data) {}
void ScreenManager::update_trajectory_tab(const telemetry::TelemetryData& data) {}
void ScreenManager::update_performance_tab(const telemetry::TelemetryData& data) {}
void ScreenManager::update_config_tab(const telemetry::TelemetryData& data) {}

} // namespace abclib