#include "abclib/configs/sd_card_utils.hpp"
#include "api.h"
#include <fstream>
#include <sstream>
#include <sys/stat.h>

namespace abclib::config {

    bool SDCardUtils::is_sd_card_available() {
        // Check if the SD card mount point exists
        struct stat info;
        return (stat(SD_ROOT, &info) == 0);
    }   

    bool SDCardUtils::file_exists(const std::string& filepath) {
        struct stat buffer;
        return (stat(filepath.c_str(), &buffer) == 0);
    }

    std::string SDCardUtils::read_file(const std::string& filepath) {
        std::ifstream file(filepath);
        
        if (!file.is_open()) {
            pros::lcd::print(0, "Failed to open: %s", filepath.c_str());
            return "";
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();
        
        return buffer.str();
    }

    bool SDCardUtils::write_file(const std::string& filepath, 
                                 const std::string& content) {
        std::ofstream file(filepath);
        
        if (!file.is_open()) {
            pros::lcd::print(0, "Failed to write: %s", filepath.c_str());
            return false;
        }
        
        file << content;
        file.close();
        
        return true;
    }

    std::string SDCardUtils::get_config_path(const std::string& robot_name) {
        return std::string(CONFIG_DIR) + robot_name + ".json";
    }

} // namespace abclib::config