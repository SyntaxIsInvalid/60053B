#include "abclib/configs/config_loader.hpp"
#include "abclib/configs/sd_card_utils.hpp"
#include "abclib/configs/config_json.hpp"
#include "abclib/configs/test_robot_defaults.hpp"
#include "abclib/configs/competition_robot_defaults.hpp"
#include "api.h"

namespace abclib::config {

    // Static member initialization
    RobotConfig ConfigLoader::cached_config_;
    bool ConfigLoader::config_loaded_ = false;

    RobotConfig ConfigLoader::load(const std::string& robot_name) {
        // If already loaded, return cached version
        if (config_loaded_) {
            return cached_config_;
        }
        
        RobotConfig config;
        
        // Try SD card first
        if (SDCardUtils::is_sd_card_available()) {
            try {
                config = load_from_sd(robot_name);
                pros::lcd::print(0, "Config: SD Card");
                pros::lcd::print(1, "%s v%s", 
                    config.robot_name.c_str(), 
                    config.version.c_str());
                
                // Cache it
                cached_config_ = config;
                config_loaded_ = true;
                return config;
                
            } catch (const std::exception& e) {
                pros::lcd::print(0, "SD Error - Hardcoded");
                pros::lcd::print(1, "%s", e.what());
                pros::delay(2000); // Show error briefly
            }
        }
        
        // Fall back to hardcoded
        config = load_hardcoded(robot_name);
        pros::lcd::print(0, "Config: Hardcoded");
        pros::lcd::print(1, "%s v%s", 
            config.robot_name.c_str(), 
            config.version.c_str());
        
        // Cache it
        cached_config_ = config;
        config_loaded_ = true;
        return config;
    }

    bool ConfigLoader::reload_from_sd(const std::string& robot_name) {
        if (!SDCardUtils::is_sd_card_available()) {
            pros::lcd::print(0, "Reload: No SD card");
            return false;
        }
        
        try {
            RobotConfig new_config = load_from_sd(robot_name);
            cached_config_ = new_config;
            
            pros::lcd::print(0, "Reloaded from SD!");
            pros::lcd::print(1, "Lat kP: %.2f", new_config.lateral_pid.kP);
            pros::lcd::print(2, "Ang kP: %.2f", new_config.angular_pid.kP);
            
            return true;
            
        } catch (const std::exception& e) {
            pros::lcd::print(0, "Reload failed");
            pros::lcd::print(1, "%s", e.what());
            return false;
        }
    }

    const RobotConfig& ConfigLoader::get_current() {
        if (!config_loaded_) {
            // This shouldn't happen, but provide a safe fallback
            cached_config_ = load_hardcoded("test_robot");
            config_loaded_ = true;
        }
        return cached_config_;
    }

    bool ConfigLoader::save_to_sd(const RobotConfig& config) {
        if (!SDCardUtils::is_sd_card_available()) {
            pros::lcd::print(0, "Save: No SD card");
            return false;
        }
        
        try {
            // Convert config to JSON
            nlohmann::json j = config;
            std::string json_str = j.dump(2); // Pretty print with 2 space indent
            
            // Write to file
            std::string filepath = SDCardUtils::get_config_path(config.robot_name);
            bool success = SDCardUtils::write_file(filepath, json_str);
            
            if (success) {
                pros::lcd::print(0, "Saved to SD!");
            } else {
                pros::lcd::print(0, "Save failed");
            }
            
            return success;
            
        } catch (const std::exception& e) {
            pros::lcd::print(0, "Save error: %s", e.what());
            return false;
        }
    }

    RobotConfig ConfigLoader::load_from_sd(const std::string& robot_name) {
        std::string filepath = SDCardUtils::get_config_path(robot_name);
        
        if (!SDCardUtils::file_exists(filepath)) {
            throw std::runtime_error("Config file not found");
        }
        
        std::string json_content = SDCardUtils::read_file(filepath);
        if (json_content.empty()) {
            throw std::runtime_error("Config file is empty");
        }
        
        return parse_json_string(json_content);
    }

    RobotConfig ConfigLoader::load_hardcoded(const std::string& robot_name) {
        if (robot_name == "test_robot") {
            return get_test_robot_defaults();
        } else if (robot_name == "competition_robot") {
            return get_competition_robot_defaults();
        } else {
            // Unknown robot - default to test robot
            pros::lcd::print(0, "Unknown: %s", robot_name.c_str());
            return get_test_robot_defaults();
        }
    }

} // namespace abclib::config