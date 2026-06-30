#include "motor/actuator.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

using Clock = std::chrono::steady_clock;
using ms    = std::chrono::milliseconds;

// ── Helpers ────────────────────────────────────────────────────────────────

const char* joint_state_name(JointState s) {
    switch (s) {
        case JointState::OFFLINE:     return "OFFLINE";
        case JointState::DISABLED:    return "DISABLED";
        case JointState::IDLE:        return "IDLE";
        case JointState::ENABLED:     return "ENABLED";
        case JointState::CALIBRATING: return "CALIBRATING";
        case JointState::FAULT:       return "FAULT";
    }
    return "UNKNOWN";
}

// ── Constructor ─────────────────────────────────────────────────────────────

Actuator::Actuator(const JointConfig& cfg)
    : cfg_(cfg)
{}

// ── State snapshot ──────────────────────────────────────────────────────────

ActuatorState Actuator::state() const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return state_;
}

// ── Rx dispatch ─────────────────────────────────────────────────────────────

void Actuator::on_rx_frame(const can_frame& frame) {
    uint32_t arb = frame.can_id & CAN_EFF_MASK;
    int func = static_cast<int>(get_func_id(arb));

    std::lock_guard<std::mutex> lk(state_mutex_);

    if (func == static_cast<int>(FuncCode::FUNC_TRANSMIT_PDO_4)) {
        // Passive broadcast: position + velocity at fast_frame_frequency Hz.
        if (frame.can_dlc < 8) return;
        float wire_pos = get_f32(frame.data);
        float vel      = get_f32(frame.data + 4);
        state_.position = wire_pos;  // firmware already subtracts position_offset in getPositionMeasured()
        state_.velocity = vel;
        state_.updated_at    = Clock::now();
        last_alive_received_ = Clock::now();

        // PDO4 from an OFFLINE motor means it power-cycled back to life — wake it.
        // Do NOT auto-advance from DISABLED: the heartbeat ACK to our own NMT DISABLED
        // would set needs_idle_wakeup_ and immediately counter the disconnect with NMT IDLE.
        // DISABLED→IDLE only happens via an explicit request_state(IDLE) (connect path).
        if (state_.joint_state == JointState::OFFLINE) {
            state_.joint_state = JointState::IDLE;
            needs_idle_wakeup_  = true;
            last_applied_config_ = std::nullopt;  // force full rewrite after power-cycle
        }

    } else if (func == static_cast<int>(FuncCode::FUNC_HEARTBEAT)) {  // NOLINT
        // 5-byte reply: mode(u8) + error(u32 LE) — new firmware.
        // Older firmware sends 8-byte zero frame.
        if (frame.can_dlc >= 1) {
            state_.mode = frame.data[0];
        }
        if (frame.can_dlc >= 5) {
            uint32_t err;
            memcpy(&err, frame.data + 1, 4);
            state_.error = err;
        }
        state_.updated_at    = Clock::now();
        last_alive_received_ = Clock::now();

        // Heartbeat from OFFLINE → motor came back after power-cycle; wake it.
        // Same reasoning as the PDO4 handler: do NOT auto-advance from DISABLED, or
        // the NMT DISABLED ACK heartbeat would undo the disconnect via needs_idle_wakeup_.
        if (state_.joint_state == JointState::OFFLINE) {
            state_.joint_state = JointState::IDLE;
            needs_idle_wakeup_   = true;
            last_applied_config_ = std::nullopt;  // force full rewrite after power-cycle
        }

        // Calibration complete when firmware returns to IDLE.
        if (state_.joint_state == JointState::CALIBRATING &&
            state_.mode == static_cast<uint8_t>(MotorMode::MODE_IDLE))  // NOLINT
        {
            state_.joint_state = JointState::IDLE;
        }

    } else if (func == static_cast<int>(FuncCode::FUNC_SYNC_EMCY)) {
        // Emergency frame: 4-byte error code.
        if (frame.can_dlc >= 4) {
            uint32_t err;
            memcpy(&err, frame.data, 4);
            state_.error |= err;
        }
        if (state_.joint_state == JointState::ENABLED ||
            state_.joint_state == JointState::CALIBRATING)
        {
            state_.joint_state = JointState::FAULT;
        }

    } else if (func == static_cast<int>(FuncCode::FUNC_TRANSMIT_PDO_2)) {
        // PDO2 feedback — unambiguous arb_id, raw [pos f32, vel f32] (firmware v3.0.7+).
        if (frame.can_dlc < 8) return;
        state_.position   = get_f32(frame.data);
        state_.velocity   = get_f32(frame.data + 4);
        state_.updated_at = Clock::now();

    } else if (func == static_cast<int>(FuncCode::FUNC_TRANSMIT_SDO)) {  // NOLINT
        // Firmware v3.0.6 and earlier send PDO2 feedback on this arb_id by mistake.
        // byte0 = 0x60 (SDO_WRITE_ACK)  → write ACK    (bytes 1-2: param_id; bytes 4-7: zeros)
        // byte0 = 0x43 (SDO_READ_RESP)  → read response (bytes 1-2: param_id; bytes 4-7: value)
        // other byte0                   → PDO2 feedback (legacy; use FUNC_TRANSMIT_PDO_2 in v3.0.7+)
        if (frame.can_dlc < 8) return;

        if (frame.data[0] == SDO_WRITE_ACK) {
            {
                std::lock_guard<std::mutex> alk(sdo_ack_mutex_);
                sdo_ack_received_ = true;
            }
            sdo_ack_cv_.notify_one();
            return;
        }
        if (frame.data[0] == SDO_READ_RESP) {
            uint16_t param_id;
            float    v;
            memcpy(&param_id, frame.data + 1, 2);
            memcpy(&v,        frame.data + 4, 4);

            // Update slow-poll telemetry fields — routed by param_id from the response.
            using P = ParamId;
            switch (static_cast<P>(param_id)) {
                case P::PARAM_POWERSTAGE_BUS_VOLTAGE_MEASURED:
                    state_.bus_voltage = (v >= 0.0f && v < 100.0f) ? v : -1.0f; break;
                case P::PARAM_CURRENT_CONTROLLER_I_Q_MEASURED:
                    state_.current = v; break;
                case P::PARAM_POSITION_CONTROLLER_TORQUE_MEASURED:
                    state_.torque  = v; break;
                default: break;
            }
            // Wake read_config_param() if it is waiting for this specific param.
            {
                std::lock_guard<std::mutex> mlk(sdo_rx_mutex_);
                sdo_rx_param_ = param_id;
                sdo_rx_value_ = v;
                sdo_rx_ready_ = true;
            }
            sdo_rx_cv_.notify_one();
            return;
        }
        // PDO2 feedback — no command-byte prefix, raw position + velocity floats.
        float wire_pos = get_f32(frame.data);
        float vel      = get_f32(frame.data + 4);
        state_.position = wire_pos;  // firmware already subtracts position_offset in getPositionMeasured()
        state_.velocity = vel;
        state_.updated_at = Clock::now();
    }
    // FUNC_TRANSMIT_PDO_3 (calibration status) — ignored for now.
}

// ── Control loop tick ───────────────────────────────────────────────────────

static void send_sdo_read(CanBusManager& bus, const std::string& channel,
                           uint8_t device_id, uint16_t param)
{
    can_frame f{};
    f.can_id  = make_arb_id(static_cast<uint8_t>(FuncCode::FUNC_RECEIVE_SDO), device_id);
    f.can_dlc = 8;
    f.data[0] = SDO_CMD_READ;       // 0x40
    f.data[1] = static_cast<uint8_t>(param & 0xFF);
    f.data[2] = static_cast<uint8_t>((param >> 8) & 0xFF);
    // bytes 3-7 stay 0
    bus.send(channel, f);
}

void Actuator::send_pdo2(CanBusManager& bus, float display_pos, float vel_ff) {
    can_frame frame{};
    frame.can_id  = make_arb_id(
        static_cast<uint8_t>(FuncCode::FUNC_RECEIVE_PDO_2),
        static_cast<uint8_t>(cfg_.device_id));
    frame.can_dlc = 8;
    put_f32(frame.data,     display_pos);  // display-frame; firmware adds position_offset internally
    put_f32(frame.data + 4, vel_ff);
    bus.send(cfg_.can_channel, frame);
}

void Actuator::tick(CanBusManager& bus) {
    // On first detection, send NMT IDLE to wake firmware from boot-time MODE_DISABLED.
    // DISABLED firmware won't respond to SDO reads, so voltage/current/torque stay zero
    // until this runs.  Safe if already IDLE: firmware treats it as a no-op.
    if (needs_idle_wakeup_) {
        needs_idle_wakeup_ = false;
        can_frame wake{};
        wake.can_id  = make_arb_id(
            static_cast<uint8_t>(FuncCode::FUNC_NMT),
            static_cast<uint8_t>(cfg_.device_id));
        wake.can_dlc = 2;
        wake.data[0] = static_cast<uint8_t>(MotorMode::MODE_IDLE);
        wake.data[1] = static_cast<uint8_t>(cfg_.device_id);
        bus.send(cfg_.can_channel, wake);
    }

    // Mark motor OFFLINE if no CAN frame (PDO4 or heartbeat) has arrived in 1500 ms.
    // 1500 ms = ~98 missed PDO4 frames at 65 Hz, or 3 missed heartbeats at 2 Hz.
    // Using last_alive_received_ (updated by both PDO4 and heartbeat) keeps motors
    // that have no PDO4 (e.g. FAST_FRAME_FREQUENCY=0 during commissioning) from
    // oscillating OFFLINE→IDLE→OFFLINE every 500 ms.
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        // DISABLED motors are intentionally silent — suppress the OFFLINE timer so
        // they don't oscillate DISABLED→OFFLINE when no frames arrive.
        if (state_.joint_state != JointState::OFFLINE &&
            state_.joint_state != JointState::DISABLED) {
            auto stale_ms = std::chrono::duration_cast<ms>(
                Clock::now() - last_alive_received_).count();
            if (stale_ms > 1500) {
                state_.joint_state    = JointState::OFFLINE;
                // firmware_version intentionally preserved — shows last known version
                // on the dashboard even during brief power cycles.  It is cleared to 0
                // only inside apply_config() after a full successful re-read, which
                // happens on the next Connect → apply_config cycle.
                last_applied_config_ = std::nullopt;  // force full reconfigure on reconnect
            }
        }
    }

    // Apply any pending state change request from the UDP command queue.
    JointState current_state;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        current_state = state_.joint_state;
    }

    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        if (state_change_pending_) {
            state_change_pending_ = false;
            JointState target = requested_state_;

            // Send NMT for state transitions that require a firmware mode change.
            can_frame nmt{};
            nmt.can_id  = make_arb_id(
                static_cast<uint8_t>(FuncCode::FUNC_NMT),
                static_cast<uint8_t>(cfg_.device_id));
            nmt.can_dlc = 2;

            bool send_nmt = false;
            if (target == JointState::ENABLED && current_state == JointState::IDLE) {
                // Pre-send NMT IDLE — firmware may still be in DISABLED at this point
                // (e.g. needs_idle_wakeup_ fired but heartbeat hasn't confirmed IDLE yet).
                // Firmware rejects DISABLED→POSITION; IDLE→POSITION always succeeds.
                can_frame pre_idle{};
                pre_idle.can_id  = make_arb_id(
                    static_cast<uint8_t>(FuncCode::FUNC_NMT),
                    static_cast<uint8_t>(cfg_.device_id));
                pre_idle.can_dlc = 2;
                pre_idle.data[0] = static_cast<uint8_t>(MotorMode::MODE_IDLE);
                pre_idle.data[1] = static_cast<uint8_t>(cfg_.device_id);
                bus.send(cfg_.can_channel, pre_idle);

                nmt.data[0] = static_cast<uint8_t>(MotorMode::MODE_POSITION);
                nmt.data[1] = static_cast<uint8_t>(cfg_.device_id);
                send_nmt = true;
                float hold;
                { std::lock_guard<std::mutex> slk(state_mutex_); hold = state_.position; }
                // Seed daemon-side position_target_ so all subsequent ticks send the
                // correct hold position until the first SET_POSITION command arrives.
                position_target_ = hold;
                // Pre-seed the FIRMWARE's position_target before entering position mode.
                // PositionController_setPositionTarget() is called unconditionally in the
                // FUNC_RECEIVE_PDO_2 handler (no mode guard), so sending PDO2 while still
                // in IDLE writes hold+offset into position_target.  When NMT POSITION then
                // fires MotorController_reset(), the target is already correct and the very
                // first control tick sees zero error.  Without this, position_target stays
                // at its init value of 0.0 for 0–1 control cycles, causing a large torque
                // spike on motors with a non-zero position_offset (e.g. left_hip_pitch).
                send_pdo2(bus, hold);
                if (send_nmt) bus.send(cfg_.can_channel, nmt);
                { std::lock_guard<std::mutex> slk(state_mutex_); state_.joint_state = JointState::ENABLED; }
                return;
            } else if (target == JointState::IDLE) {
                nmt.data[0] = static_cast<uint8_t>(MotorMode::MODE_IDLE);
                nmt.data[1] = static_cast<uint8_t>(cfg_.device_id);
                send_nmt = true;
            } else if (target == JointState::DISABLED) {
                nmt.data[0] = static_cast<uint8_t>(MotorMode::MODE_DISABLED);
                nmt.data[1] = static_cast<uint8_t>(cfg_.device_id);
                send_nmt = true;
            } else if (target == JointState::CALIBRATING && current_state == JointState::IDLE) {
                nmt.data[0] = static_cast<uint8_t>(MotorMode::MODE_CALIBRATION);
                nmt.data[1] = static_cast<uint8_t>(cfg_.device_id);
                send_nmt = true;
            }

            if (send_nmt) {
                bus.send(cfg_.can_channel, nmt);
                std::lock_guard<std::mutex> slk(state_mutex_);
                state_.joint_state = target;
                current_state = target;
            }
        }
    }

    if (current_state == JointState::ENABLED) {
        float target;
        { std::lock_guard<std::mutex> lk(cmd_mutex_); target = position_target_; }
        send_pdo2(bus, target);

    }
    // IDLE, OFFLINE, CALIBRATING, FAULT: no outbound frames from tick().
    // NOTE: Do NOT send FUNC_HEARTBEAT (0x700+device_id) to IDLE motors.
    // That arb_id is used by the motor itself to BROADCAST its heartbeat.
    // Sending on the same arb_id from the host causes CAN bus collisions that
    // accumulate error counts on both sides and can corrupt adjacent SDO frames.
    // The firmware watchdog does not fire in IDLE mode, so no feed is needed.

    // Slow-poll telemetry via SDO reads (~3 Hz per field at 200 Hz / 60 ticks).
    // Skipped when slow_poll_enabled_ is false (e.g. during Flash Wizard commissioning)
    // to prevent SDO read responses from consuming generic_listener_ futures.
    if (slow_poll_enabled_.load(std::memory_order_relaxed) &&
        (current_state == JointState::ENABLED || current_state == JointState::IDLE)) {
        using P = ParamId;
        slow_poll_counter_++;
        switch (slow_poll_counter_ % 60) {
            case 0:
                send_sdo_read(bus, cfg_.can_channel, cfg_.device_id,
                    static_cast<uint16_t>(P::PARAM_POWERSTAGE_BUS_VOLTAGE_MEASURED));     break;
            case 20:
                send_sdo_read(bus, cfg_.can_channel, cfg_.device_id,
                    static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_Q_MEASURED));     break;
            case 40:
                send_sdo_read(bus, cfg_.can_channel, cfg_.device_id,
                    static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_TORQUE_MEASURED)); break;
            default: break;
        }
    }
}

// ── Setters (UDP thread) ────────────────────────────────────────────────────

void Actuator::request_state(JointState s) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    requested_state_    = s;
    state_change_pending_ = true;
}

void Actuator::set_position_target(float pos_rad) {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    position_target_ = pos_rad;
}

void Actuator::clear_fault(CanBusManager& bus) {
    using P = ParamId;
    // Zero the error register on the hardware so slow-poll reads don't re-surface the fault.
    sdo_write_u32(bus, static_cast<uint16_t>(P::PARAM_ERROR), 0, 200);
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_.error       = 0;
        state_.joint_state = JointState::IDLE;
    }
}

// ── Blocking SDO write helpers ──────────────────────────────────────────────

static can_frame make_sdo_write(int device_id, uint16_t param, uint8_t cmd,
                                const uint8_t* data4)
{
    can_frame frame{};
    frame.can_id  = make_arb_id(
        static_cast<uint8_t>(FuncCode::FUNC_RECEIVE_SDO),
        static_cast<uint8_t>(device_id));
    frame.can_dlc = 8;
    frame.data[0] = cmd;
    frame.data[1] = static_cast<uint8_t>(param & 0xFF);
    frame.data[2] = static_cast<uint8_t>((param >> 8) & 0xFF);
    frame.data[3] = 0;
    memcpy(frame.data + 4, data4, 4);
    return frame;
}

bool Actuator::sdo_write_f32(CanBusManager& bus, uint16_t param, float val, int timeout_ms) {
    uint8_t raw[4];
    memcpy(raw, &val, 4);
    { std::lock_guard<std::mutex> lk(sdo_ack_mutex_); sdo_ack_received_ = false; }
    auto frame = make_sdo_write(cfg_.device_id, param, SDO_CMD_WRITE, raw);
    bus.send(cfg_.can_channel, frame);
    std::unique_lock<std::mutex> lk(sdo_ack_mutex_);
    return sdo_ack_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
        [this]{ return sdo_ack_received_; });
}

bool Actuator::sdo_write_u32(CanBusManager& bus, uint16_t param, uint32_t val, int timeout_ms) {
    uint8_t raw[4];
    memcpy(raw, &val, 4);
    { std::lock_guard<std::mutex> lk(sdo_ack_mutex_); sdo_ack_received_ = false; }
    auto frame = make_sdo_write(cfg_.device_id, param, SDO_CMD_WRITE, raw);
    bus.send(cfg_.can_channel, frame);
    std::unique_lock<std::mutex> lk(sdo_ack_mutex_);
    return sdo_ack_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
        [this]{ return sdo_ack_received_; });
}

bool Actuator::sdo_write_i32(CanBusManager& bus, uint16_t param, int32_t val, int timeout_ms) {
    uint8_t raw[4];
    memcpy(raw, &val, 4);
    { std::lock_guard<std::mutex> lk(sdo_ack_mutex_); sdo_ack_received_ = false; }
    auto frame = make_sdo_write(cfg_.device_id, param, SDO_CMD_WRITE, raw);
    bus.send(cfg_.can_channel, frame);
    std::unique_lock<std::mutex> lk(sdo_ack_mutex_);
    return sdo_ack_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
        [this]{ return sdo_ack_received_; });
}

// ── update_cfg ───────────────────────────────────────────────────────────────

void Actuator::update_cfg(const nlohmann::json& j) {
    auto f = [&](const char* k, float cur) -> float {
        return j.contains(k) && j[k].is_number() ? j[k].get<float>() : cur;
    };
    auto i = [&](const char* k, int cur) -> int {
        return j.contains(k) && j[k].is_number() ? j[k].get<int>() : cur;
    };
    auto b = [&](const char* k, bool cur) -> bool {
        return j.contains(k) && j[k].is_boolean() ? j[k].get<bool>() : cur;
    };

    cfg_.gear_ratio               = f("gear_ratio",               cfg_.gear_ratio);
    cfg_.position_kp              = f("position_kp",              cfg_.position_kp);
    cfg_.position_ki              = f("position_ki",              cfg_.position_ki);
    cfg_.velocity_kp              = f("velocity_kp",              cfg_.velocity_kp);
    cfg_.velocity_ki              = f("velocity_ki",              cfg_.velocity_ki);
    cfg_.torque_limit             = f("torque_limit",             cfg_.torque_limit);
    cfg_.velocity_limit           = f("velocity_limit",           cfg_.velocity_limit);
    cfg_.position_limit_min       = f("position_limit_min",       cfg_.position_limit_min);
    cfg_.position_limit_max       = f("position_limit_max",       cfg_.position_limit_max);
    cfg_.position_offset          = f("position_offset",          cfg_.position_offset);
    cfg_.torque_filter_alpha      = f("torque_filter_alpha",      cfg_.torque_filter_alpha);
    cfg_.current_limit            = f("current_limit",            cfg_.current_limit);
    cfg_.current_kp               = f("current_kp",               cfg_.current_kp);
    cfg_.current_ki               = f("current_ki",               cfg_.current_ki);
    cfg_.undervoltage_threshold   = f("undervoltage_threshold",   cfg_.undervoltage_threshold);
    cfg_.overvoltage_threshold    = f("overvoltage_threshold",    cfg_.overvoltage_threshold);
    cfg_.bus_voltage_filter_alpha = f("bus_voltage_filter_alpha", cfg_.bus_voltage_filter_alpha);
    cfg_.torque_constant          = f("torque_constant",          cfg_.torque_constant);
    cfg_.max_calibration_current  = f("max_calibration_current",  cfg_.max_calibration_current);
    cfg_.encoder_position_offset  = f("encoder_position_offset",  cfg_.encoder_position_offset);
    cfg_.velocity_filter_alpha    = f("velocity_filter_alpha",    cfg_.velocity_filter_alpha);
    cfg_.electrical_offset        = f("electrical_offset",        cfg_.electrical_offset);
    cfg_.fast_frame_frequency     = i("fast_frame_frequency",     cfg_.fast_frame_frequency);
    cfg_.watchdog_timeout_ms      = i("watchdog_timeout",         cfg_.watchdog_timeout_ms);
    cfg_.pole_pairs               = i("pole_pairs",               cfg_.pole_pairs);
    cfg_.cpr                      = i("cpr",                      cfg_.cpr);
    cfg_.phase_inverted           = b("phase_inverted",           cfg_.phase_inverted);
}

// ── apply_config ────────────────────────────────────────────────────────────

bool Actuator::apply_config(CanBusManager& bus, int timeout_ms) {
    using P = ParamId;
    const JointConfig& c = cfg_;
    const JointConfig* prev = last_applied_config_.has_value() ? &(*last_applied_config_) : nullptr;

    // Read firmware version on every fresh connect (first apply_config after OFFLINE).
    // prev==nullptr means last_applied_config_ was nullopt — this IS a fresh connect.
    // This re-reads even if the cached value is non-zero, so a reflash is picked up.
    bool needs_fw_read = (prev == nullptr);
    if (needs_fw_read) {
        float fw_f = read_config_param(bus, static_cast<uint16_t>(P::PARAM_FIRMWARE_VERSION), 300);
        if (!std::isnan(fw_f)) {
            uint32_t fw_u32;
            std::memcpy(&fw_u32, &fw_f, 4);
            std::lock_guard<std::mutex> lk(state_mutex_);
            state_.firmware_version = fw_u32;
        }
    }

    // Limits are stored in display frame (user-facing rad) but the firmware compares them
    // against position_target = (display_command + position_offset), i.e. the internal frame.
    // Adjust by adding position_offset so the firmware's clamp operates in the correct frame.
    // Pre-compute offset-adjusted values and previous-run equivalents for correct delta detection.
    const float lim_min_adj  = c.position_limit_min + c.position_offset;
    const float lim_max_adj  = c.position_limit_max + c.position_offset;
    const float prev_lim_min = prev ? (prev->position_limit_min + prev->position_offset) : 0.0f;
    const float prev_lim_max = prev ? (prev->position_limit_max + prev->position_offset) : 0.0f;

    struct F32Entry { uint16_t param; float val; const float* prev_val; };
    const F32Entry f32[] = {
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_GEAR_RATIO),            c.gear_ratio,               prev ? &prev->gear_ratio               : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_KP),           c.position_kp,              prev ? &prev->position_kp              : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_KI),           c.position_ki,              prev ? &prev->position_ki              : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_KP),           c.velocity_kp,              prev ? &prev->velocity_kp              : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_KI),           c.velocity_ki,              prev ? &prev->velocity_ki              : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_TORQUE_LIMIT),          c.torque_limit,             prev ? &prev->torque_limit             : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT),        c.velocity_limit,           prev ? &prev->velocity_limit           : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER),  lim_min_adj,                prev ? &prev_lim_min                   : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER),  lim_max_adj,                prev ? &prev_lim_max                   : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_OFFSET),       c.position_offset,          prev ? &prev->position_offset          : nullptr},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA),   c.torque_filter_alpha,      prev ? &prev->torque_filter_alpha      : nullptr},
        {static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_LIMIT),                c.current_limit,            prev ? &prev->current_limit            : nullptr},
        {static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_KP),                   c.current_kp,               prev ? &prev->current_kp               : nullptr},
        {static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_KI),                   c.current_ki,               prev ? &prev->current_ki               : nullptr},
        {static_cast<uint16_t>(P::PARAM_POWERSTAGE_UNDERVOLTAGE_THRESHOLD),         c.undervoltage_threshold,   prev ? &prev->undervoltage_threshold   : nullptr},
        {static_cast<uint16_t>(P::PARAM_POWERSTAGE_OVERVOLTAGE_THRESHOLD),          c.overvoltage_threshold,    prev ? &prev->overvoltage_threshold    : nullptr},
        {static_cast<uint16_t>(P::PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA),       c.bus_voltage_filter_alpha, prev ? &prev->bus_voltage_filter_alpha : nullptr},
        {static_cast<uint16_t>(P::PARAM_MOTOR_TORQUE_CONSTANT),                     c.torque_constant,          prev ? &prev->torque_constant          : nullptr},
        {static_cast<uint16_t>(P::PARAM_MOTOR_MAX_CALIBRATION_CURRENT),             c.max_calibration_current,  prev ? &prev->max_calibration_current  : nullptr},
        {static_cast<uint16_t>(P::PARAM_ENCODER_POSITION_OFFSET),                   c.encoder_position_offset,  prev ? &prev->encoder_position_offset  : nullptr},
        {static_cast<uint16_t>(P::PARAM_ENCODER_VELOCITY_FILTER_ALPHA),             c.velocity_filter_alpha,    prev ? &prev->velocity_filter_alpha    : nullptr},
        {static_cast<uint16_t>(P::PARAM_ENCODER_FLUX_OFFSET),                       c.electrical_offset,        prev ? &prev->electrical_offset        : nullptr},
    };

    for (auto& e : f32) {
        if (e.prev_val && fabsf(e.val - *e.prev_val) < 1e-6f) continue;
        if (!sdo_write_f32(bus, e.param, e.val, timeout_ms)) {
            fprintf(stderr, "[Actuator] apply_config: f32 SDO write 0x%03X failed for %s\n",
                    e.param, cfg_.name.c_str());
            return false;
        }
    }

    struct U32Entry { uint16_t param; uint32_t val; uint32_t prev_val; bool has_prev; };
    const U32Entry u32[] = {
        {static_cast<uint16_t>(P::PARAM_FAST_FRAME_FREQUENCY), static_cast<uint32_t>(c.fast_frame_frequency),
            prev ? static_cast<uint32_t>(prev->fast_frame_frequency) : 0u, prev != nullptr},
        {static_cast<uint16_t>(P::PARAM_WATCHDOG_TIMEOUT),     static_cast<uint32_t>(c.watchdog_timeout_ms),
            prev ? static_cast<uint32_t>(prev->watchdog_timeout_ms)  : 0u, prev != nullptr},
        {static_cast<uint16_t>(P::PARAM_MOTOR_POLE_PAIRS),     static_cast<uint32_t>(c.pole_pairs),
            prev ? static_cast<uint32_t>(prev->pole_pairs)           : 0u, prev != nullptr},
        {static_cast<uint16_t>(P::PARAM_ENCODER_CPR),          static_cast<uint32_t>(c.cpr),
            prev ? static_cast<uint32_t>(prev->cpr)                  : 0u, prev != nullptr},
    };
    for (auto& e : u32) {
        if (e.has_prev && e.val == e.prev_val) continue;
        if (!sdo_write_u32(bus, e.param, e.val, timeout_ms)) {
            fprintf(stderr, "[Actuator] apply_config: u32 SDO write 0x%03X failed for %s\n",
                    e.param, cfg_.name.c_str());
            return false;
        }
    }

    // phase_order: -1 if phase_inverted, else +1
    int32_t phase_order = c.phase_inverted ? -1 : 1;
    bool phase_unchanged = prev && (c.phase_inverted == prev->phase_inverted);
    if (!phase_unchanged) {
        if (!sdo_write_i32(bus, static_cast<uint16_t>(P::PARAM_MOTOR_PHASE_ORDER), phase_order, timeout_ms)) {
            fprintf(stderr, "[Actuator] apply_config: phase_order SDO write failed for %s\n",
                    cfg_.name.c_str());
            return false;
        }
    }

    last_applied_config_ = cfg_;
    fprintf(stderr, "[Actuator] Config applied: %s (id=%d)\n", cfg_.name.c_str(), c.device_id);
    return true;
}

// ── write_gains ──────────────────────────────────────────────────────────────

bool Actuator::write_gains(CanBusManager& bus, float kp, float ki, float velocity_kp,
                           float torque_limit, int timeout_ms) {
    using P = ParamId;
    if (!sdo_write_f32(bus, static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_KP),
                       kp, timeout_ms)) {
        fprintf(stderr, "[Actuator] write_gains: Kp write failed for %s\n", cfg_.name.c_str());
        return false;
    }
    if (!sdo_write_f32(bus, static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_KI),
                       ki, timeout_ms)) {
        fprintf(stderr, "[Actuator] write_gains: Ki write failed for %s\n", cfg_.name.c_str());
        return false;
    }
    if (!sdo_write_f32(bus, static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_KP),
                       velocity_kp, timeout_ms)) {
        fprintf(stderr, "[Actuator] write_gains: velocity_kp write failed for %s\n", cfg_.name.c_str());
        return false;
    }
    if (!sdo_write_f32(bus, static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_TORQUE_LIMIT),
                       torque_limit, timeout_ms)) {
        fprintf(stderr, "[Actuator] write_gains: torque_limit write failed for %s\n", cfg_.name.c_str());
        return false;
    }
    return true;
}

// ── store_to_flash ───────────────────────────────────────────────────────────

void Actuator::store_to_flash(CanBusManager& bus) {
    can_frame f{};
    f.can_id  = make_arb_id(static_cast<uint8_t>(FuncCode::FUNC_FLASH),
                             static_cast<uint8_t>(cfg_.device_id));
    f.can_dlc = 1;
    f.data[0] = 0x01;  // store to flash
    bus.send(cfg_.can_channel, f);
    // Flash write takes up to ~100 ms; sleep before returning so caller can rely on completion.
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    fprintf(stderr, "[Actuator] Stored to flash: %s\n", cfg_.name.c_str());
}

// ── read_config_param ────────────────────────────────────────────────────────

float Actuator::read_config_param(CanBusManager& bus, uint16_t param, int timeout_ms) {
    {
        std::lock_guard<std::mutex> lk(sdo_rx_mutex_);
        sdo_rx_ready_ = false;
        sdo_rx_param_ = 0;
    }
    send_sdo_read(bus, cfg_.can_channel, cfg_.device_id, param);
    std::unique_lock<std::mutex> lk(sdo_rx_mutex_);
    bool ok = sdo_rx_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
        [this, param]{ return sdo_rx_ready_ && sdo_rx_param_ == param; });
    return ok ? sdo_rx_value_ : std::numeric_limits<float>::quiet_NaN();
}
