#pragma once
#include <string>

namespace abclib::config {

    class SDCardUtils {
    public:
        // Check if SD card is mounted and accessible
        static bool is_sd_card_available();
        
        // Check if a specific file exists
        static bool file_exists(const std::string& filepath);
        
        // Read entire file contents as string
        static std::string read_file(const std::string& filepath);
        
        // Write string to file (overwrites if exists)
        static bool write_file(const std::string& filepath, 
                              const std::string& content);
        
        // Get full path to config file
        static std::string get_config_path(const std::string& robot_name);
        
    private:
        static constexpr const char* SD_ROOT = "/usd/";
        static constexpr const char* CONFIG_DIR = "/usd/configs/";
    };

} // namespace abclib::config