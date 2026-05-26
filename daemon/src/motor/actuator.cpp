#include "motor/actuator.hpp"

#include <cstdio>
#include <cstring>
#include <thread>

using Clock = std::chrono::steady_clock;
using ms    = std::chrono::milliseconds;

// ── Helpers ────────────────────────────────────────────────────────────────

const char* joint_state_name(JointState s) {
    switch (s) {
        case JointState::OFFLINE:     return "OFFLINE";
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
        state_.position = wire_pos;
        state_.velocity = vel;
        state_.updated_at = Clock::now();
        last_pdo4_received_ = Clock::now();

        // Seeing PDO4 means the device is alive — advance from OFFLINE.
        if (state_.joint_state == JointState::OFFLINE)
            state_.joint_state = JointState::IDLE;

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
        state_.updated_at = Clock::now();

        // Device is alive.
        if (state_.joint_state == JointState::OFFLINE)
            state_.joint_state = JointState::IDLE;

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

    } else if (func == static_cast<int>(FuncCode::FUNC_TRANSMIT_SDO)) {  // NOLINT
        // Three frame types share this function code:
        //   DLC=1, data[0]=0x60 → SDO write ACK
        //   DLC=4             → SDO read response (value in bytes 0-3, no param echo)
        //   DLC=8             → PDO2 position+velocity response
        if (frame.can_dlc >= 1 && frame.data[0] == SDO_WRITE_ACK) {
            return;  // write ACK — apply_config handles this via its own wait loop
        }
        if (frame.can_dlc == 4) {
            // Recoil firmware SDO read response: raw float in bytes 0-3, no param_id echo.
            float v = get_f32(frame.data);
            if (sdo_config_active_.load(std::memory_order_relaxed)) {
                // read_config_param() is waiting — wake it with this value.
                {
                    std::lock_guard<std::mutex> mlk(sdo_rx_mutex_);
                    sdo_rx_value_ = v;
                    sdo_rx_ready_ = true;
                }
                sdo_rx_cv_.notify_one();
            } else {
                // Slow-poll response — route by the last param we requested.
                using P = ParamId;
                switch (static_cast<P>(sdo_pending_param_.load(std::memory_order_relaxed))) {
                    case P::PARAM_POWERSTAGE_BUS_VOLTAGE_MEASURED:
                        state_.bus_voltage = (v >= 0.0f && v < 100.0f) ? v : -1.0f; break;
                    case P::PARAM_CURRENT_CONTROLLER_I_Q_MEASURED:
                        state_.current = v; break;
                    case P::PARAM_POSITION_CONTROLLER_TORQUE_MEASURED:
                        state_.torque = v; break;
                    default: break;
                }
            }
            return;
        }
        if (frame.can_dlc >= 8) {
            // PDO2 response: position + velocity.
            float wire_pos = get_f32(frame.data);
            float vel      = get_f32(frame.data + 4);
            state_.position = wire_pos;
            state_.velocity = vel;
            state_.updated_at = Clock::now();
        }
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
    put_f32(frame.data,     display_pos);
    put_f32(frame.data + 4, vel_ff);
    bus.send(cfg_.can_channel, frame);
}

void Actuator::tick(CanBusManager& bus) {
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
                nmt.data[0] = static_cast<uint8_t>(MotorMode::MODE_POSITION);
                nmt.data[1] = static_cast<uint8_t>(cfg_.device_id);
                send_nmt = true;
                // Send hold position immediately after NMT so firmware doesn't snap to 0.
                float hold;
                { std::lock_guard<std::mutex> slk(state_mutex_); hold = state_.position; }
                if (send_nmt) bus.send(cfg_.can_channel, nmt);
                send_pdo2(bus, hold);
                { std::lock_guard<std::mutex> slk(state_mutex_); state_.joint_state = JointState::ENABLED; }
                return;
            } else if (target == JointState::IDLE) {
                nmt.data[0] = static_cast<uint8_t>(MotorMode::MODE_IDLE);
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

    auto now = Clock::now();

    if (current_state == JointState::ENABLED) {
        float target;
        { std::lock_guard<std::mutex> lk(cmd_mutex_); target = position_target_; }
        send_pdo2(bus, target);

    } else if (current_state == JointState::IDLE) {
        // Feed watchdog via HEARTBEAT every 200 ms.
        auto elapsed = std::chrono::duration_cast<ms>(now - last_heartbeat_sent_);
        if (elapsed.count() >= 200) {
            can_frame hb{};
            hb.can_id  = make_arb_id(
                static_cast<uint8_t>(FuncCode::FUNC_HEARTBEAT),
                static_cast<uint8_t>(cfg_.device_id));
            hb.can_dlc = 8;
            memset(hb.data, 0, 8);
            bus.send(cfg_.can_channel, hb);
            last_heartbeat_sent_ = now;
        }
    }
    // OFFLINE, CALIBRATING, FAULT: no outbound frames from tick().

    // Slow-poll telemetry via SDO reads (~3 Hz per field, one per 60 ticks at 200 Hz).
    // Paused while read_config_param() is in progress to avoid response crosstalk.
    if ((current_state == JointState::ENABLED || current_state == JointState::IDLE) &&
        !sdo_config_active_.load(std::memory_order_relaxed))
    {
        using P = ParamId;
        slow_poll_counter_++;
        uint16_t poll_param = 0;
        switch (slow_poll_counter_ % 60) {
            case 0:  poll_param = static_cast<uint16_t>(P::PARAM_POWERSTAGE_BUS_VOLTAGE_MEASURED);    break;
            case 20: poll_param = static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_Q_MEASURED);    break;
            case 40: poll_param = static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_TORQUE_MEASURED); break;
            default: break;
        }
        if (poll_param != 0) {
            sdo_pending_param_.store(poll_param, std::memory_order_relaxed);
            send_sdo_read(bus, cfg_.can_channel, cfg_.device_id, poll_param);
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

void Actuator::clear_fault() {
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_.error       = 0;
        state_.joint_state = JointState::IDLE;
    }
}

// ── Blocking SDO write helpers ──────────────────────────────────────────────

// Wait for an SDO write ACK (0x60 on TRANSMIT_SDO) or a PDO2 response that
// confirms the write. Drains the named bus until ACK or timeout.
static bool wait_for_sdo_ack(CanBusManager& bus, const std::string& channel,
                              int device_id, int timeout_ms)
{
    auto deadline = Clock::now() + ms(timeout_ms);
    while (Clock::now() < deadline) {
        // drain_all with a lambda; spin-read is acceptable only at startup.
        bool got = false;
        bus.drain_all([&](const std::string& ifname, const can_frame& f) {
            if (ifname != channel) return;
            uint32_t arb  = f.can_id & CAN_EFF_MASK;
            int func      = static_cast<int>(get_func_id(arb));
            int dev       = static_cast<int>(get_device_id(arb));
            if (dev != device_id) return;
            if (func == static_cast<int>(FuncCode::FUNC_TRANSMIT_SDO) &&
                f.can_dlc >= 1 && f.data[0] == SDO_WRITE_ACK)
            {
                got = true;
            }
        });
        if (got) return true;
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    return false;
}

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
    auto frame = make_sdo_write(cfg_.device_id, param, SDO_CMD_WRITE, raw);
    bus.send(cfg_.can_channel, frame);
    return wait_for_sdo_ack(bus, cfg_.can_channel, cfg_.device_id, timeout_ms);
}

bool Actuator::sdo_write_u32(CanBusManager& bus, uint16_t param, uint32_t val, int timeout_ms) {
    uint8_t raw[4];
    memcpy(raw, &val, 4);
    auto frame = make_sdo_write(cfg_.device_id, param, SDO_CMD_WRITE, raw);
    bus.send(cfg_.can_channel, frame);
    return wait_for_sdo_ack(bus, cfg_.can_channel, cfg_.device_id, timeout_ms);
}

bool Actuator::sdo_write_i32(CanBusManager& bus, uint16_t param, int32_t val, int timeout_ms) {
    uint8_t raw[4];
    memcpy(raw, &val, 4);
    auto frame = make_sdo_write(cfg_.device_id, param, SDO_CMD_WRITE, raw);
    bus.send(cfg_.can_channel, frame);
    return wait_for_sdo_ack(bus, cfg_.can_channel, cfg_.device_id, timeout_ms);
}

// ── apply_config ────────────────────────────────────────────────────────────

bool Actuator::apply_config(CanBusManager& bus, int timeout_ms) {
    using P = ParamId;
    const JointConfig& c = cfg_;
    int d = c.device_id;

    struct F32Entry { uint16_t param; float val; };
    const F32Entry f32[] = {
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_GEAR_RATIO),            c.gear_ratio},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_KP),           c.position_kp},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_KI),           c.position_ki},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_KP),           c.velocity_kp},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_KI),           c.velocity_ki},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_TORQUE_LIMIT),          c.torque_limit},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT),        c.velocity_limit},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER),  c.position_limit_min},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER),  c.position_limit_max},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_OFFSET),       c.position_offset},
        {static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA),   c.torque_filter_alpha},
        {static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_LIMIT),                c.current_limit},
        {static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_KP),                   c.current_kp},
        {static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_KI),                   c.current_ki},
        {static_cast<uint16_t>(P::PARAM_POWERSTAGE_UNDERVOLTAGE_THRESHOLD),         c.undervoltage_threshold},
        {static_cast<uint16_t>(P::PARAM_POWERSTAGE_OVERVOLTAGE_THRESHOLD),          c.overvoltage_threshold},
        {static_cast<uint16_t>(P::PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA),       c.bus_voltage_filter_alpha},
        {static_cast<uint16_t>(P::PARAM_MOTOR_TORQUE_CONSTANT),                     c.torque_constant},
        {static_cast<uint16_t>(P::PARAM_MOTOR_MAX_CALIBRATION_CURRENT),             c.max_calibration_current},
        {static_cast<uint16_t>(P::PARAM_ENCODER_POSITION_OFFSET),                   c.encoder_position_offset},
        {static_cast<uint16_t>(P::PARAM_ENCODER_VELOCITY_FILTER_ALPHA),             c.velocity_filter_alpha},
        {static_cast<uint16_t>(P::PARAM_ENCODER_FLUX_OFFSET),                       c.electrical_offset},
    };

    for (auto& e : f32) {
        if (!sdo_write_f32(bus, e.param, e.val, timeout_ms)) {
            fprintf(stderr, "[Actuator] apply_config: f32 SDO write 0x%03X failed for %s\n",
                    e.param, cfg_.name.c_str());
            return false;
        }
    }

    struct U32Entry { uint16_t param; uint32_t val; };
    const U32Entry u32[] = {
        {static_cast<uint16_t>(P::PARAM_FAST_FRAME_FREQUENCY), static_cast<uint32_t>(c.fast_frame_frequency)},
        {static_cast<uint16_t>(P::PARAM_WATCHDOG_TIMEOUT),     static_cast<uint32_t>(c.watchdog_timeout_ms)},
        {static_cast<uint16_t>(P::PARAM_MOTOR_POLE_PAIRS),     static_cast<uint32_t>(c.pole_pairs)},
        {static_cast<uint16_t>(P::PARAM_ENCODER_CPR),          static_cast<uint32_t>(c.cpr)},
    };
    for (auto& e : u32) {
        if (!sdo_write_u32(bus, e.param, e.val, timeout_ms)) {
            fprintf(stderr, "[Actuator] apply_config: u32 SDO write 0x%03X failed for %s\n",
                    e.param, cfg_.name.c_str());
            return false;
        }
    }

    // phase_order: -1 if phase_inverted, else +1
    int32_t phase_order = c.phase_inverted ? -1 : 1;
    if (!sdo_write_i32(bus, static_cast<uint16_t>(P::PARAM_MOTOR_PHASE_ORDER), phase_order, timeout_ms)) {
        fprintf(stderr, "[Actuator] apply_config: phase_order SDO write failed for %s\n",
                cfg_.name.c_str());
        return false;
    }

    fprintf(stderr, "[Actuator] Config applied: %s (id=%d)\n", cfg_.name.c_str(), d);
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
    // Pause slow-poll so its in-flight response can't be mistaken for ours
    // (firmware sends a bare 4-byte value with no param_id echo).
    sdo_config_active_.store(true, std::memory_order_seq_cst);
    // Wait one control-loop tick for any already-sent slow-poll response to arrive.
    std::this_thread::sleep_for(std::chrono::milliseconds(6));
    {
        std::lock_guard<std::mutex> lk(sdo_rx_mutex_);
        sdo_rx_ready_ = false;  // discard any stale/spurious signal from above
    }
    send_sdo_read(bus, cfg_.can_channel, cfg_.device_id, param);
    std::unique_lock<std::mutex> lk(sdo_rx_mutex_);
    bool ok = sdo_rx_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
        [this]{ return sdo_rx_ready_; });
    float result = ok ? sdo_rx_value_ : std::numeric_limits<float>::quiet_NaN();
    sdo_config_active_.store(false, std::memory_order_seq_cst);
    return result;
}
