#pragma once
#include "config.hpp"
#include "external/json.hpp"

namespace abclib::config {

    // Forward declarations for nlohmann conversion functions
    void to_json(nlohmann::json& j, const MotorConfig& m);
    void from_json(const nlohmann::json& j, MotorConfig& m);
    
    void to_json(nlohmann::json& j, const PIDConfig& p);
    void from_json(const nlohmann::json& j, PIDConfig& p);
    
    void to_json(nlohmann::json& j, const TurnInPlaceFF& t);
    void from_json(const nlohmann::json& j, TurnInPlaceFF& t);
    
    void to_json(nlohmann::json& j, const SettlementConfig& s);
    void from_json(const nlohmann::json& j, SettlementConfig& s);
    
    void to_json(nlohmann::json& j, const RobotConfig& r);
    void from_json(const nlohmann::json& j, RobotConfig& r);
    
    // Helper to parse JSON string into RobotConfig
    RobotConfig parse_json_string(const std::string& json_string);

} // namespace abclib::config