#include "config/config_loader.hpp"

#include <fstream>
#include <stdexcept>

#include "json.hpp"

using json = nlohmann::json;

// min null → -NO_LIMIT_RAD, max null → +NO_LIMIT_RAD, matching Python's lower_bound/upper_bound.
static float limit_or_sentinel_min(const json& val) {
    if (val.is_null()) return -NO_LIMIT_RAD;
    return val.get<float>();
}
static float limit_or_sentinel_max(const json& val) {
    if (val.is_null()) return NO_LIMIT_RAD;
    return val.get<float>();
}

RobotConfig load_config(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("config_loader: cannot open " + path);

    json root;
    try {
        f >> root;
    } catch (const json::parse_error& e) {
        throw std::runtime_error(std::string("config_loader: JSON parse error: ") + e.what());
    }

    RobotConfig cfg;
    cfg.robot_name   = root.at("robot_name").get<std::string>();
    cfg.telemetry_hz = root.value("telemetry_hz", 10);

    // can_assignments is optional
    if (root.contains("can_assignments") && root["can_assignments"].is_object()) {
        for (auto& [serial, label] : root["can_assignments"].items())
            cfg.can_assignments[serial] = label.get<std::string>();
    }

    const json& joints_obj = root.at("joints");
    cfg.joints.reserve(joints_obj.size());

    for (auto& [key, j] : joints_obj.items()) {
        JointConfig jc;
        jc.name       = j.at("joint_name").get<std::string>();
        jc.can_channel = j.at("can_channel").get<std::string>();
        jc.device_id  = j.at("can_id").get<int>();

        jc.phase_inverted            = j.value("phase_inverted", false);
        jc.electrical_offset         = j.value("electrical_offset", 0.0f);
        jc.position_offset           = j.value("position_offset", 0.0f);
        jc.encoder_position_offset   = j.value("encoder_position_offset", 0.0f);
        jc.gear_ratio                = j.at("gear_ratio").get<float>();

        const json& lim = j.at("position_limits");
        jc.position_limit_min = limit_or_sentinel_min(lim.at("min"));
        jc.position_limit_max = limit_or_sentinel_max(lim.at("max"));

        jc.position_kp  = j.value("position_kp", 0.0f);
        jc.position_ki  = j.value("position_ki", 0.0f);
        jc.velocity_kp  = j.value("velocity_kp", 0.0f);
        jc.velocity_ki  = j.value("velocity_ki", 0.0f);
        jc.current_kp   = j.value("current_kp",  0.0f);
        jc.current_ki   = j.value("current_ki",  0.0f);

        jc.torque_limit   = j.value("torque_limit",   0.0f);
        jc.velocity_limit = j.value("velocity_limit", 0.0f);
        jc.current_limit  = j.value("current_limit",  0.0f);

        jc.torque_filter_alpha      = j.value("torque_filter_alpha",      0.0f);
        jc.velocity_filter_alpha    = j.value("velocity_filter_alpha",    0.0f);
        jc.bus_voltage_filter_alpha = j.value("bus_voltage_filter_alpha", 0.0f);

        jc.undervoltage_threshold = j.value("undervoltage_threshold", 0.0f);
        jc.overvoltage_threshold  = j.value("overvoltage_threshold",  0.0f);

        jc.pole_pairs              = j.value("pole_pairs", 0);
        jc.torque_constant         = j.value("torque_constant", 0.0f);
        jc.max_calibration_current = j.value("max_calibration_current", 0.0f);
        jc.cpr                     = j.value("cpr", 0);

        jc.watchdog_timeout_ms   = j.value("watchdog_timeout",     1000);
        jc.fast_frame_frequency  = j.value("fast_frame_frequency", 0);

        cfg.joints.push_back(std::move(jc));
    }

    return cfg;
}
