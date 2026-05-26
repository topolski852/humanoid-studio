#pragma once

// Loads humanoid_lite.json into C++ structs using nlohmann/json.
// Null position_limits become ±100.0 rad (matches Python _NO_LIMIT_RAD sentinel).

#include <string>
#include <vector>
#include <unordered_map>

static constexpr float NO_LIMIT_RAD = 100.0f;

struct JointConfig {
    std::string name;
    std::string can_channel;
    int         device_id;

    bool        phase_inverted;
    float       electrical_offset;
    float       position_offset;
    float       encoder_position_offset;
    float       gear_ratio;

    // Position limits (NO_LIMIT_RAD if null in JSON)
    float       position_limit_min;
    float       position_limit_max;

    // Control gains
    float       position_kp;
    float       position_ki;
    float       velocity_kp;
    float       velocity_ki;
    float       current_kp;
    float       current_ki;

    // Limits
    float       torque_limit;
    float       velocity_limit;
    float       current_limit;

    // Filter alphas
    float       torque_filter_alpha;
    float       velocity_filter_alpha;
    float       bus_voltage_filter_alpha;

    // Voltage thresholds
    float       undervoltage_threshold;
    float       overvoltage_threshold;

    // Motor parameters
    int         pole_pairs;
    float       torque_constant;
    float       max_calibration_current;
    int         cpr;

    // Watchdog / streaming
    int         watchdog_timeout_ms;
    int         fast_frame_frequency;
};

struct RobotConfig {
    std::string                                   robot_name;
    int                                           telemetry_hz;
    std::vector<JointConfig>                      joints;
    std::unordered_map<std::string, std::string>  can_assignments; // serial → bus label
};

// Throws std::runtime_error on missing required fields or file not found.
RobotConfig load_config(const std::string& path);
