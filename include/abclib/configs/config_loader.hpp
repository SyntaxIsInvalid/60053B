#pragma once
#include "config.hpp"
#include <string>

namespace abclib::config {

    class ConfigLoader {
    public:
        // Primary load function - tries SD card, falls back to hardcoded
        static RobotConfig load(const std::string& robot_name);
        
        // Force reload from SD card (for live tuning)
        static bool reload_from_sd(const std::string& robot_name);
        
        // Get the currently loaded config (cached)
        static const RobotConfig& get_current();
        
        // Save current config to SD card (for backup/export)
        static bool save_to_sd(const RobotConfig& config);
        
    private:
        static RobotConfig load_from_sd(const std::string& robot_name);
        static RobotConfig load_hardcoded(const std::string& robot_name);
        
        static RobotConfig cached_config_;
        static bool config_loaded_;
    };

} // namespace abclib::config