#pragma once
#include "liblvgl/lvgl.h"
#include "abclib/telemetry/telemetry.hpp"
#include <vector>

namespace abclib {

class ScreenManager {
private:
    // Tabview and tabs
    lv_obj_t* tabview;
    lv_obj_t* tab_overview;
    lv_obj_t* tab_pid;
    lv_obj_t* tab_path;
    lv_obj_t* tab_performance;
    lv_obj_t* tab_image;
    
    // Labels for each tab (created once, updated repeatedly)
    std::vector<lv_obj_t*> overview_labels;
    std::vector<lv_obj_t*> pid_labels;
    std::vector<lv_obj_t*> path_labels;
    std::vector<lv_obj_t*> performance_labels;
    
    // Full-screen image object (container)
    lv_obj_t* image_obj;
    
    // Tab creation helpers
    void create_overview_tab();
    void create_pid_tab();
    void create_path_tab();
    void create_performance_tab();
    void create_image_tab();
    
    // Update helpers
    void update_overview_tab(const TelemetryData& data);
    void update_pid_tab(const TelemetryData& data);
    void update_path_tab(const TelemetryData& data);
    void update_performance_tab(const TelemetryData& data);
    
    // Image display helpers
    void show_fullscreen_image();
    void hide_fullscreen_image();
    
public:
    ScreenManager() = default;
    
    // Initialize all tabs and UI elements
    void initialize();
    
    // Update telemetry display (called from loop)
    void update_telemetry(const TelemetryData& data);
};

} // namespace abclib