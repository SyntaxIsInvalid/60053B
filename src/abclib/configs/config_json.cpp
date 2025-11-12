#include "abclib/configs/config_json.hpp"
#include "api.h"

namespace abclib::config {

    using json = nlohmann::json;

    // MotorConfig conversions
    void to_json(json& j, const MotorConfig& m) {
        j = json{
            {"kS", m.kS},
            {"kV", m.kV},
            {"kA", m.kA},
            {"kPv", m.kPv},
            {"kIv", m.kIv},
            {"kDv", m.kDv},
            {"enable_voltage_compensation", m.enable_voltage_compensation},
            {"compensation_nominal", m.compensation_nominal},
            {"compensation_min_battery", m.compensation_min_battery}
        };
    }

    void from_json(const json& j, MotorConfig& m) {
        j.at("kS").get_to(m.kS);
        j.at("kV").get_to(m.kV);
        j.at("kA").get_to(m.kA);
        j.at("kPv").get_to(m.kPv);
        j.at("kIv").get_to(m.kIv);
        j.at("kDv").get_to(m.kDv);
        j.at("enable_voltage_compensation").get_to(m.enable_voltage_compensation);
        j.at("compensation_nominal").get_to(m.compensation_nominal);
        j.at("compensation_min_battery").get_to(m.compensation_min_battery);
    }

    // PIDConfig conversions
    void to_json(json& j, const PIDConfig& p) {
        j = json{
            {"kP", p.kP},
            {"kI", p.kI},
            {"kD", p.kD}
        };
    }

    void from_json(const json& j, PIDConfig& p) {
        j.at("kP").get_to(p.kP);
        j.at("kI").get_to(p.kI);
        j.at("kD").get_to(p.kD);
    }

    // TurnInPlaceFF conversions
    void to_json(json& j, const TurnInPlaceFF& t) {
        j = json{
            {"kS", t.kS},
            {"kV", t.kV},
            {"kA", t.kA}
        };
    }

    void from_json(const json& j, TurnInPlaceFF& t) {
        j.at("kS").get_to(t.kS);
        j.at("kV").get_to(t.kV);
        j.at("kA").get_to(t.kA);
    }

    // SettlementConfig conversions
    void to_json(json& j, const SettlementConfig& s) {
        j = json{
            {"angular_threshold_deg", s.angular_threshold_deg},
            {"position_threshold_inches", s.position_threshold_inches},
            {"angular_velocity_threshold", s.angular_velocity_threshold},
            {"linear_velocity_threshold", s.linear_velocity_threshold},
            {"settle_count_required", s.settle_count_required}
        };
    }

    void from_json(const json& j, SettlementConfig& s) {
        j.at("angular_threshold_deg").get_to(s.angular_threshold_deg);
        j.at("position_threshold_inches").get_to(s.position_threshold_inches);
        j.at("angular_velocity_threshold").get_to(s.angular_velocity_threshold);
        j.at("linear_velocity_threshold").get_to(s.linear_velocity_threshold);
        j.at("settle_count_required").get_to(s.settle_count_required);
    }

    // RobotConfig conversions
    void to_json(json& j, const RobotConfig& r) {
        j = json{
            {"robot_name", r.robot_name},
            {"version", r.version},
            {"left_motor", r.left_motor},
            {"right_motor", r.right_motor},
            {"lateral_pid", r.lateral_pid},
            {"angular_pid", r.angular_pid},
            {"profiled_turn_pid", r.profiled_turn_pid},
            {"turn_in_place_ff", r.turn_in_place_ff},
            {"settlement", r.settlement}
        };
    }

    void from_json(const json& j, RobotConfig& r) {
        j.at("robot_name").get_to(r.robot_name);
        j.at("version").get_to(r.version);
        j.at("left_motor").get_to(r.left_motor);
        j.at("right_motor").get_to(r.right_motor);
        j.at("lateral_pid").get_to(r.lateral_pid);
        j.at("angular_pid").get_to(r.angular_pid);
        j.at("profiled_turn_pid").get_to(r.profiled_turn_pid);
        j.at("turn_in_place_ff").get_to(r.turn_in_place_ff);
        j.at("settlement").get_to(r.settlement);
    }

    // Helper function to parse JSON string
    RobotConfig parse_json_string(const std::string& json_string) {
        try {
            json j = json::parse(json_string);
            RobotConfig config = j.get<RobotConfig>();
            config.source = ConfigSource::SD_CARD;
            return config;
        } catch (const json::exception& e) {
            pros::lcd::print(0, "JSON parse error");
            pros::lcd::print(1, "%s", e.what());
            throw; // Re-throw so caller can handle
        }
    }

} // namespace abclib::config