#pragma once

// Per-joint state machine and CAN frame handler.
//
// Threading: state_ is protected by state_mutex_. The control loop calls
// on_rx_frame() and tick() from one thread; the UDP server reads state()
// from another. apply_config() is blocking and must only be called before
// the control loop starts.

#include "config/config_loader.hpp"
#include "can/can_bus_manager.hpp"
#include "motor/recoil_protocol.hpp"
#include "json.hpp"

#include <chrono>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <string>

enum class JointState {
    OFFLINE,
    IDLE,
    ENABLED,
    CALIBRATING,
    FAULT,
};

const char* joint_state_name(JointState s);

struct ActuatorState {
    JointState  joint_state   = JointState::OFFLINE;
    float       position      = 0.0f;   // display-frame rad (wire - offset)
    float       velocity      = 0.0f;   // rad/s
    float       torque        = 0.0f;   // Nm estimated
    float       current       = 0.0f;   // Iq A
    uint8_t     mode          = 0;      // firmware MotorMode
    uint32_t    error         = 0;      // firmware error bitmask
    float       bus_voltage   = -1.0f;  // V; -1 = unknown
    std::chrono::steady_clock::time_point updated_at{};
};

class Actuator {
public:
    explicit Actuator(const JointConfig& cfg);

    const std::string&  name()      const { return cfg_.name; }
    int                 device_id() const { return cfg_.device_id; }
    const std::string&  can_channel() const { return cfg_.can_channel; }
    const JointConfig&  config()    const { return cfg_; }

    // Thread-safe state snapshot for the UDP server.
    ActuatorState state() const;

    // Called from the control loop Rx dispatch path (on_frame callback).
    void on_rx_frame(const can_frame& frame);

    // Called once per control-loop tick (200 Hz).
    // Sends PDO2 (ENABLED) or HEARTBEAT (IDLE, every 200 ms).
    void tick(CanBusManager& bus);

    // --- Thread-safe setters called from UDP server command queue ---

    // Request a mode transition on the next tick.
    void request_state(JointState s);

    // Set position target (display-frame rad).
    void set_position_target(float pos_rad);

    // Clear fault and return to IDLE.
    void clear_fault();

    // Update the in-memory JointConfig from a flat JSON object.
    // Fields absent from the JSON are left unchanged.
    // Not thread-safe against simultaneous control-loop reads of cfg_ —
    // callers (robot.cpp APPLY_CONFIG handler) must only call this before apply_config().
    void update_cfg(const nlohmann::json& j);

    // --- Blocking SDO write sequence (startup only, not called during real-time loop) ---
    // Returns false if any write fails or times out.
    bool apply_config(CanBusManager& bus, int timeout_ms = 500);

    // Send FUNC_FLASH store command; blocks ~150 ms for flash write to complete.
    void store_to_flash(CanBusManager& bus);

    // Blocking SDO read for a single parameter. Returns NaN on timeout.
    float read_config_param(CanBusManager& bus, uint16_t param, int timeout_ms = 300);

private:
    // Build and send one PDO2. pos is display-frame; firmware applies position_offset internally.
    void send_pdo2(CanBusManager& bus, float display_pos, float vel_ff = 0.0f);

    // Blocking SDO write with ACK wait. Returns false on timeout.
    bool sdo_write_f32(CanBusManager& bus, uint16_t param, float val, int timeout_ms);
    bool sdo_write_u32(CanBusManager& bus, uint16_t param, uint32_t val, int timeout_ms);
    bool sdo_write_i32(CanBusManager& bus, uint16_t param, int32_t val, int timeout_ms);

    JointConfig cfg_;

    mutable std::mutex state_mutex_;
    ActuatorState      state_;

    // Command staging (written by UDP thread, read by control loop).
    std::mutex    cmd_mutex_;
    JointState    requested_state_   = JointState::OFFLINE;
    bool          state_change_pending_ = false;
    float         position_target_   = 0.0f;   // display-frame

    std::chrono::steady_clock::time_point last_heartbeat_sent_{};
    std::chrono::steady_clock::time_point last_pdo4_received_{};

    // Slow-poll counter for periodic SDO telemetry reads (bus_voltage, current, torque).
    uint32_t slow_poll_counter_ = 0;

    // SDO write ACK mailbox: on_rx_frame() signals here; sdo_write_*() waits.
    std::mutex              sdo_ack_mutex_;
    std::condition_variable sdo_ack_cv_;
    bool                    sdo_ack_received_ = false;

    // SDO read response mailbox: on_rx_frame() writes here; read_config_param() waits.
    // param_id is echoed in the standard 8-byte response, enabling param-matched waits.
    std::mutex              sdo_rx_mutex_;
    std::condition_variable sdo_rx_cv_;
    bool                    sdo_rx_ready_  = false;
    uint16_t                sdo_rx_param_  = 0;
    float                   sdo_rx_value_  = 0.0f;
};
