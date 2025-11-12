#include "screen_manager.hpp"
#include <cstdio>

namespace abclib {

void ScreenManager::initialize() {
    // Create the tabview
    tabview = lv_tabview_create(lv_screen_active());
    
    // Create all tabs
    tab_overview = lv_tabview_add_tab(tabview, "Overview");
    tab_pid = lv_tabview_add_tab(tabview, "PID");
    tab_path = lv_tabview_add_tab(tabview, "Path");
    tab_performance = lv_tabview_add_tab(tabview, "Performance");
    tab_image = lv_tabview_add_tab(tabview, "Image");
    
    // Setup all tabs
    create_overview_tab();
    create_pid_tab();
    create_path_tab();
    create_performance_tab();
    create_image_tab();
}

void ScreenManager::create_overview_tab() {
    // Create labels for overview data
    // We'll create 3 labels for now: pose, velocity, battery
    for (int i = 0; i < 3; i++) {
        lv_obj_t* label = lv_label_create(tab_overview);
        lv_label_set_text(label, "Loading...");
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 10 + i * 40);  // More spacing
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);  // Allow wrapping
        lv_obj_set_width(label, 450);  // Set width for wrapping
        overview_labels.push_back(label);
    }
}

void ScreenManager::create_pid_tab() {
    // Just a placeholder label for now
    lv_obj_t* label = lv_label_create(tab_pid);
    lv_label_set_text(label, "PID Control Data\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_path_tab() {
    // Just a placeholder label for now
    lv_obj_t* label = lv_label_create(tab_path);
    lv_label_set_text(label, "Path Tracking Data\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_performance_tab() {
    // Just a placeholder label for now
    lv_obj_t* label = lv_label_create(tab_performance);
    lv_label_set_text(label, "Performance Metrics\n(Not implemented yet)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

void ScreenManager::create_image_tab() {
    // Create a button to show the image
    lv_obj_t* btn = lv_button_create(tab_image);
    lv_obj_set_size(btn, 200, 60);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    
    lv_obj_t* btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "View Image");
    lv_obj_center(btn_label);
    
    // Add click event to show full-screen image
    lv_obj_add_event_cb(btn, [](lv_event_t* e) {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->show_fullscreen_image();
    }, LV_EVENT_CLICKED, this);
    
    // Create the full-screen image container (initially hidden)
    image_obj = lv_obj_create(lv_screen_active());
    lv_obj_set_size(image_obj, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(image_obj, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(image_obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(image_obj, 0, 0);
    lv_obj_set_style_pad_all(image_obj, 0, 0);
    
    // Create the actual image inside the container
    lv_obj_t* img = lv_image_create(image_obj);
    lv_image_set_src(img, "S:/images/deft.bin");
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    
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
    lv_obj_move_to_index(image_obj, -1);  // Move to front (-1 means top)
}


void ScreenManager::hide_fullscreen_image() {
    lv_obj_add_flag(image_obj, LV_OBJ_FLAG_HIDDEN);
}

void ScreenManager::update_telemetry(const TelemetryData& data) {
    // Only update overview tab for now
    update_overview_tab(data);
}

void ScreenManager::update_overview_tab(const TelemetryData& data) {
    // Update the 3 labels we created
    if (overview_labels.size() >= 3) {
        // Create buffers for formatted strings
        char buf0[64];
        char buf1[64];
        char buf2[64];
        
        // Label 0: Pose
        snprintf(buf0, sizeof(buf0), "X:%.2f Y:%.2f Th:%.1f",
            data.pose.x(),
            data.pose.y(),
            data.pose.theta() * 180.0 / M_PI);
        lv_label_set_text(overview_labels[0], buf0);
        
        // Label 1: Velocity
        snprintf(buf1, sizeof(buf1), "V:%.2f W:%.2f",
            data.pose_v.inches_per_sec,
            data.pose_omega.rad_per_sec);
        lv_label_set_text(overview_labels[1], buf1);
        
        // Label 2: Battery
        snprintf(buf2, sizeof(buf2), "%.2fV %.0f%% [%s%.2fx]",
            data.battery_voltage.volts,
            data.battery_capacity_percent,
            data.voltage_compensation_active ? "C" : "-",
            data.voltage_compensation_scale);
        lv_label_set_text(overview_labels[2], buf2);
    }
}

// Stub implementations for update functions we haven't implemented yet
void ScreenManager::update_pid_tab(const TelemetryData& data) {}
void ScreenManager::update_path_tab(const TelemetryData& data) {}
void ScreenManager::update_performance_tab(const TelemetryData& data) {}

} // namespace abclib