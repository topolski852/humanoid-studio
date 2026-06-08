#include "control/robot.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

#include "json.hpp"

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using ms    = std::chrono::milliseconds;

// ── Constructor / destructor ─────────────────────────────────────────────────

Robot::Robot(RobotConfig cfg, RobotOptions opts)
    : cfg_(std::move(cfg))
    , opts_(opts)
    , udp_server_(opts_.cmd_port)
    , broadcaster_("127.0.0.1", opts_.telemetry_port)
{}

Robot::~Robot() {
    stop();
}

// ── start() ──────────────────────────────────────────────────────────────────

bool Robot::start() {
    // Collect unique CAN interface names.
    std::vector<std::string> ifnames;
    for (const auto& jc : cfg_.joints) {
        bool found = false;
        for (const auto& n : ifnames) if (n == jc.can_channel) { found = true; break; }
        if (!found) ifnames.push_back(jc.can_channel);
    }

    bus_mgr_ = std::make_unique<CanBusManager>(ifnames);

    // Build Actuator objects.
    for (const auto& jc : cfg_.joints) {
        auto a = std::make_unique<Actuator>(jc);
        actuator_by_name_[jc.name] = a.get();
        actuators_.push_back(std::move(a));
    }

    // Wire UDP server.
    udp_server_.set_handler([this](const std::string& req) {
        return handle_command(req);
    });

    if (!udp_server_.start()) {
        fprintf(stderr, "[Robot] UDP server failed to start\n");
        return false;
    }

    running_ = true;

    // Start telemetry thread.
    telemetry_thread_ = std::thread(&Robot::telemetry_loop, this);

    // Start 200 Hz control loop.
    ControlLoop::Options cl_opts;
    cl_opts.period_s    = 1.0 / 200.0;
    cl_opts.sched_prio  = opts_.control_prio;
    cl_opts.cpu_affinity = opts_.control_cpu;
    cl_opts.name        = "control";
    control_loop_.start([this]{ control_tick(); }, cl_opts);

    fprintf(stderr, "[Robot] started — %zu joints, %zu buses\n",
            actuators_.size(), ifnames.size());
    return true;
}

// ── stop() ───────────────────────────────────────────────────────────────────

void Robot::stop() {
    if (!running_.exchange(false)) return;

    // Stop control loop first so no more frames are sent.
    control_loop_.stop();

    // Damp all enabled joints.
    for (auto& a : actuators_) {
        auto s = a->state();
        if (s.joint_state == JointState::ENABLED ||
            s.joint_state == JointState::CALIBRATING)
        {
            can_frame nmt{};
            nmt.can_id  = make_arb_id(static_cast<uint8_t>(FuncCode::FUNC_NMT),
                                      static_cast<uint8_t>(a->device_id()));
            nmt.can_dlc = 2;
            nmt.data[0] = static_cast<uint8_t>(MotorMode::MODE_DAMPING);
            nmt.data[1] = static_cast<uint8_t>(a->device_id());
            bus_mgr_->send(a->can_channel(), nmt);
        }
    }
    std::this_thread::sleep_for(ms(500));

    // Idle all joints.
    for (auto& a : actuators_) {
        can_frame nmt{};
        nmt.can_id  = make_arb_id(static_cast<uint8_t>(FuncCode::FUNC_NMT),
                                  static_cast<uint8_t>(a->device_id()));
        nmt.can_dlc = 2;
        nmt.data[0] = static_cast<uint8_t>(MotorMode::MODE_IDLE);
        nmt.data[1] = static_cast<uint8_t>(a->device_id());
        bus_mgr_->send(a->can_channel(), nmt);
    }
    std::this_thread::sleep_for(ms(200));

    udp_server_.stop();
    if (telemetry_thread_.joinable()) telemetry_thread_.join();

    fprintf(stderr, "[Robot] stopped\n");
}

// ── Control tick (200 Hz) ────────────────────────────────────────────────────

void Robot::control_tick() {
    // 1. Drain all pending Rx frames and dispatch to actuators + generic listeners.
    bus_mgr_->drain_all([this](const std::string& ifname, const can_frame& frame) {
        uint32_t arb = frame.can_id & CAN_EFF_MASK;
        int dev_id = static_cast<int>(get_device_id(arb));
        for (auto& a : actuators_) {
            if (a->device_id() == dev_id && a->can_channel() == ifname) {
                a->on_rx_frame(frame);
                break;
            }
        }
        generic_listener_.on_frame(ifname, frame);
    });

    // 2. Tick each actuator (send PDO2 or heartbeat).
    for (auto& a : actuators_) {
        a->tick(*bus_mgr_);
    }
}

// ── Telemetry loop ───────────────────────────────────────────────────────────

void Robot::telemetry_loop() {
    int hz = (cfg_.telemetry_hz > 0 && cfg_.telemetry_hz <= 100)
             ? cfg_.telemetry_hz : opts_.telemetry_hz;
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz));
    auto next_wake = Clock::now() + period;

    while (running_.load()) {
        std::string payload = build_telemetry_json(telemetry_seq_++);
        broadcaster_.send(payload);

        auto now = Clock::now();
        if (now < next_wake) std::this_thread::sleep_for(next_wake - now);
        next_wake += period;
    }
}

// ── Telemetry JSON builder ───────────────────────────────────────────────────

std::string Robot::build_telemetry_json(uint64_t seq) {
    json j;
    j["type"] = "TELEMETRY";
    j["seq"]  = seq;
    j["timestamp_us"] = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now().time_since_epoch()).count();

    json joints_obj = json::object();
    for (auto& a : actuators_) {
        ActuatorState s = a->state();
        json jj;
        jj["state"]    = joint_state_name(s.joint_state);
        jj["position"] = s.position;
        jj["velocity"] = s.velocity;
        jj["torque"]   = s.torque;
        jj["current"]  = s.current;
        jj["mode"]     = static_cast<int>(s.mode);
        jj["error"]    = s.error;
        jj["bus_voltage"] = (s.bus_voltage >= 0.0f)
                            ? json(s.bus_voltage) : json(nullptr);
        joints_obj[a->name()] = std::move(jj);
    }
    j["joints"] = std::move(joints_obj);

    // Bus health.
    json buses_obj = json::object();
    for (auto& [name, stats] : bus_mgr_->stats()) {
        json bj;
        bj["open"]       = stats.open;
        bj["tx_dropped"] = stats.tx_dropped;
        bj["rx_frames"]  = stats.rx_frames;
        buses_obj[name]  = std::move(bj);
    }
    j["bus_health"] = std::move(buses_obj);

    return j.dump();
}

// ── Command handler ──────────────────────────────────────────────────────────

std::string Robot::handle_command(const std::string& request) {
    json req, resp;
    try {
        req = json::parse(request);
    } catch (...) {
        return R"({"type":"ERROR","msg":"JSON parse error"})";
    }

    std::string type = req.value("type", "");
    std::string id   = req.value("id", "");

    auto ack = [&]() -> std::string {
        return json{{"type", "ACK"}, {"id", id}}.dump();
    };
    auto error = [&](const std::string& msg) -> std::string {
        return json{{"type", "ERROR"}, {"id", id}, {"msg", msg}}.dump();
    };

    if (type == "PING") {
        return json{{"type", "PONG"}, {"id", id}, {"daemon_version", "1.0"}}.dump();
    }

    if (type == "GET_STATE") {
        std::string name = req.value("joint_name", "");
        auto it = actuator_by_name_.find(name);
        if (it == actuator_by_name_.end()) return error("unknown joint: " + name);
        ActuatorState s = it->second->state();
        json r;
        r["type"] = "STATE";
        r["id"]   = id;
        r["state"]["position"]   = s.position;
        r["state"]["velocity"]   = s.velocity;
        r["state"]["torque"]     = s.torque;
        r["state"]["current"]    = s.current;
        r["state"]["mode"]       = static_cast<int>(s.mode);
        r["state"]["error"]      = s.error;
        r["state"]["joint_state"] = joint_state_name(s.joint_state);
        r["state"]["bus_voltage"] = (s.bus_voltage >= 0.0f)
                                    ? json(s.bus_voltage) : json(nullptr);
        return r.dump();
    }

    if (type == "GET_ALL_STATES") {
        json r;
        r["type"] = "ALL_STATES";
        r["id"]   = id;
        r["states"] = json::object();
        for (auto& a : actuators_) {
            ActuatorState s = a->state();
            json jj;
            jj["position"]   = s.position;
            jj["velocity"]   = s.velocity;
            jj["torque"]     = s.torque;
            jj["current"]    = s.current;
            jj["mode"]       = static_cast<int>(s.mode);
            jj["error"]      = s.error;
            jj["joint_state"] = joint_state_name(s.joint_state);
            jj["bus_voltage"] = (s.bus_voltage >= 0.0f)
                                ? json(s.bus_voltage) : json(nullptr);
            r["states"][a->name()] = std::move(jj);
        }
        return r.dump();
    }

    if (type == "SET_MODE") {
        std::string name = req.value("joint_name", "");
        std::string mode = req.value("mode", "");
        auto it = actuator_by_name_.find(name);
        if (it == actuator_by_name_.end()) return error("unknown joint: " + name);
        if (mode == "POSITION" || mode == "ENABLED") {
            it->second->request_state(JointState::ENABLED);
        } else if (mode == "IDLE" || mode == "DISABLED") {
            it->second->request_state(JointState::IDLE);
        } else {
            return error("unknown mode: " + mode);
        }
        return ack();
    }

    if (type == "SET_ALL_MODE") {
        std::string mode = req.value("mode", "");
        JointState target;
        if (mode == "POSITION" || mode == "ENABLED") {
            target = JointState::ENABLED;
        } else if (mode == "IDLE" || mode == "DISABLED") {
            target = JointState::IDLE;
        } else {
            return error("unknown mode: " + mode);
        }
        for (auto& a : actuators_) a->request_state(target);
        return ack();
    }

    if (type == "SET_POSITION") {
        std::string name = req.value("joint_name", "");
        float pos = req.value("position_rad", 0.0f);
        auto it = actuator_by_name_.find(name);
        if (it == actuator_by_name_.end()) return error("unknown joint: " + name);
        it->second->set_position_target(pos);
        return ack();
    }

    if (type == "CLEAR_ERROR") {
        std::string name = req.value("joint_name", "");
        auto it = actuator_by_name_.find(name);
        if (it == actuator_by_name_.end()) return error("unknown joint: " + name);
        it->second->clear_fault();
        return ack();
    }

    if (type == "APPLY_CONFIG") {
        std::string name = req.value("joint_name", "");
        auto it = actuator_by_name_.find(name);
        if (it == actuator_by_name_.end()) return error("unknown joint: " + name);
        bool ok = it->second->apply_config(*bus_mgr_);
        return ok ? ack() : error("apply_config failed for " + name);
    }

    if (type == "APPLY_ALL_CONFIGS") {
        for (auto& a : actuators_) {
            if (!a->apply_config(*bus_mgr_))
                fprintf(stderr, "[Robot] apply_config failed for %s\n", a->name().c_str());
        }
        return ack();
    }

    if (type == "STORE_TO_FLASH") {
        std::string name = req.value("joint_name", "");
        auto it = actuator_by_name_.find(name);
        if (it == actuator_by_name_.end()) return error("unknown joint: " + name);
        it->second->store_to_flash(*bus_mgr_);
        return ack();
    }

    if (type == "READ_CONFIG") {
        std::string name = req.value("joint_name", "");
        auto it = actuator_by_name_.find(name);
        if (it == actuator_by_name_.end()) return error("unknown joint: " + name);

        using P = ParamId;
        static const std::vector<std::pair<std::string, uint16_t>> PARAMS = {
            {"gear_ratio",               static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_GEAR_RATIO)},
            {"position_kp",              static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_KP)},
            {"position_ki",              static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_KI)},
            {"velocity_kp",              static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_KP)},
            {"velocity_ki",              static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_KI)},
            {"torque_limit",             static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_TORQUE_LIMIT)},
            {"velocity_limit",           static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT)},
            {"position_limit_lower",     static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER)},
            {"position_limit_upper",     static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER)},
            {"position_offset",          static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_POSITION_OFFSET)},
            {"torque_filter_alpha",      static_cast<uint16_t>(P::PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA)},
            {"current_limit",            static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_LIMIT)},
            {"current_kp",               static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_KP)},
            {"current_ki",               static_cast<uint16_t>(P::PARAM_CURRENT_CONTROLLER_I_KI)},
            {"undervoltage_threshold",   static_cast<uint16_t>(P::PARAM_POWERSTAGE_UNDERVOLTAGE_THRESHOLD)},
            {"overvoltage_threshold",    static_cast<uint16_t>(P::PARAM_POWERSTAGE_OVERVOLTAGE_THRESHOLD)},
            {"bus_voltage_filter_alpha", static_cast<uint16_t>(P::PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA)},
            {"torque_constant",          static_cast<uint16_t>(P::PARAM_MOTOR_TORQUE_CONSTANT)},
            {"encoder_position_offset",  static_cast<uint16_t>(P::PARAM_ENCODER_POSITION_OFFSET)},
            {"velocity_filter_alpha",    static_cast<uint16_t>(P::PARAM_ENCODER_VELOCITY_FILTER_ALPHA)},
            {"electrical_offset",        static_cast<uint16_t>(P::PARAM_ENCODER_FLUX_OFFSET)},
            {"fast_frame_frequency",     static_cast<uint16_t>(P::PARAM_FAST_FRAME_FREQUENCY)},
            {"watchdog_timeout",         static_cast<uint16_t>(P::PARAM_WATCHDOG_TIMEOUT)},
        };

        nlohmann::json cfg_j;
        for (auto& [key, param_id] : PARAMS) {
            float v = it->second->read_config_param(*bus_mgr_, param_id, 300);
            cfg_j[key] = std::isnan(v) ? nlohmann::json(nullptr) : nlohmann::json(v);
        }
        nlohmann::json resp;
        resp["type"]   = "CONFIG";
        resp["id"]     = id;
        resp["config"] = cfg_j;
        return resp.dump();
    }

    if (type == "SHUTDOWN") {
        // Respond first, then stop asynchronously.
        std::thread([this]{ std::this_thread::sleep_for(ms(100)); stop(); }).detach();
        return ack();
    }

    // ── Generic CAN passthrough commands ─────────────────────────────────────
    // These allow the Flash Wizard (and any other Python caller) to perform
    // SDO reads/writes, heartbeat waits, bus sniffs, and calibration against
    // arbitrary device IDs without opening a second SocketCAN socket.

    if (type == "GENERIC_SDO_WRITE") {
        std::string channel  = req.value("channel", "");
        int         device_id = req.value("device_id", -1);
        int         param_id  = req.value("param_id", -1);
        std::string val_type  = req.value("value_type", "u32");
        int         timeout_ms = req.value("timeout_ms", 500);

        if (!bus_mgr_->is_open(channel))
            return error("bus not open: " + channel);
        if (device_id < 0 || device_id > 127 || param_id < 0)
            return error("invalid device_id or param_id");

        uint8_t val_bytes[4] = {};
        if (val_type == "f32") {
            float v = req.value("value", 0.0f);
            std::memcpy(val_bytes, &v, 4);
        } else if (val_type == "i32") {
            int32_t v = req.value("value", 0);
            std::memcpy(val_bytes, &v, 4);
        } else {
            uint32_t v = req.value<uint32_t>("value", 0u);
            std::memcpy(val_bytes, &v, 4);
        }

        auto fut = generic_listener_.expect_once(channel,
            static_cast<uint8_t>(device_id), FUNC_TRANSMIT_SDO);

        can_frame f{};
        f.can_id  = make_arb_id(FUNC_RECEIVE_SDO, static_cast<uint8_t>(device_id));
        f.can_dlc = 8;
        f.data[0] = SDO_CMD_WRITE;
        f.data[1] = static_cast<uint8_t>(param_id & 0xFF);
        f.data[2] = static_cast<uint8_t>((param_id >> 8) & 0xFF);
        f.data[3] = 0;
        std::memcpy(f.data + 4, val_bytes, 4);
        bus_mgr_->send(channel, f);

        auto stat = fut.wait_for(ms(timeout_ms));
        if (stat != std::future_status::ready) {
            return json{{"type","SDO_WRITE_RESULT"},{"id",id},{"status","NO_ACK"}}.dump();
        }
        can_frame ack_frame = fut.get();
        if (ack_frame.can_dlc < 1 || ack_frame.data[0] != SDO_WRITE_ACK) {
            return json{{"type","SDO_WRITE_RESULT"},{"id",id},{"status","BAD_ACK"}}.dump();
        }
        return json{{"type","SDO_WRITE_RESULT"},{"id",id},{"status","OK"}}.dump();
    }

    if (type == "GENERIC_SDO_READ") {
        std::string channel  = req.value("channel", "");
        int         device_id = req.value("device_id", -1);
        int         param_id  = req.value("param_id", -1);
        int         timeout_ms = req.value("timeout_ms", 500);

        if (!bus_mgr_->is_open(channel))
            return error("bus not open: " + channel);
        if (device_id < 0 || device_id > 127 || param_id < 0)
            return error("invalid device_id or param_id");

        auto fut = generic_listener_.expect_once(channel,
            static_cast<uint8_t>(device_id), FUNC_TRANSMIT_SDO);

        can_frame f{};
        f.can_id  = make_arb_id(FUNC_RECEIVE_SDO, static_cast<uint8_t>(device_id));
        f.can_dlc = 8;
        f.data[0] = SDO_CMD_READ;
        f.data[1] = static_cast<uint8_t>(param_id & 0xFF);
        f.data[2] = static_cast<uint8_t>((param_id >> 8) & 0xFF);
        std::memset(f.data + 3, 0, 5);
        bus_mgr_->send(channel, f);

        auto stat = fut.wait_for(ms(timeout_ms));
        if (stat != std::future_status::ready) {
            return json{{"type","SDO_READ_RESULT"},{"id",id},{"status","TIMEOUT"}}.dump();
        }
        can_frame rx = fut.get();

        // Production firmware: 8-byte response, value at bytes 4-7.
        // Commissioning ELF v3.0.0: 4-byte response, value at bytes 0-3.
        uint8_t vbytes[4] = {};
        if (rx.can_dlc >= 8 && rx.data[0] == SDO_READ_RESP) {
            std::memcpy(vbytes, rx.data + 4, 4);
        } else if (rx.can_dlc >= 4) {
            std::memcpy(vbytes, rx.data, 4);
        } else {
            return json{{"type","SDO_READ_RESULT"},{"id",id},{"status","BAD_RESPONSE"}}.dump();
        }

        uint32_t u32 = 0; float f32 = 0.0f; int32_t i32 = 0;
        std::memcpy(&u32, vbytes, 4);
        std::memcpy(&f32, vbytes, 4);
        std::memcpy(&i32, vbytes, 4);

        return json{{"type","SDO_READ_RESULT"},{"id",id},{"status","OK"},
                    {"value_u32",u32},{"value_f32",f32},{"value_i32",i32}}.dump();
    }

    if (type == "WAIT_HEARTBEAT") {
        std::string channel  = req.value("channel", "");
        int         device_id = req.value("device_id", -1);
        int         timeout_ms = req.value("timeout_ms", 3000);

        if (!bus_mgr_->is_open(channel))
            return error("bus not open: " + channel);
        if (device_id < 0 || device_id > 127)
            return error("invalid device_id");

        auto fut = generic_listener_.expect_once(channel,
            static_cast<uint8_t>(device_id), FUNC_HEARTBEAT);

        auto stat = fut.wait_for(ms(timeout_ms));
        if (stat != std::future_status::ready) {
            return json{{"type","HEARTBEAT_RESULT"},{"id",id},{"status","TIMEOUT"}}.dump();
        }
        can_frame hb = fut.get();
        uint8_t  mode = (hb.can_dlc >= 1) ? hb.data[0] : 0;
        uint32_t err  = 0;
        if (hb.can_dlc >= 5) std::memcpy(&err, hb.data + 1, 4);
        return json{{"type","HEARTBEAT_RESULT"},{"id",id},{"status","OK"},
                    {"mode",mode},{"error",err}}.dump();
    }

    if (type == "SNIFF_BUS") {
        std::string channel    = req.value("channel", "");
        int         duration_ms = req.value("duration_ms", 3000);
        if (duration_ms > 30000) duration_ms = 30000;

        if (!bus_mgr_->is_open(channel))
            return error("bus not open: " + channel);

        uint64_t sniff_id = generic_listener_.begin_sniff(channel);
        std::this_thread::sleep_for(ms(duration_ms));
        auto frames = generic_listener_.end_sniff(sniff_id);

        json frames_arr = json::array();
        for (const auto& fr : frames) {
            uint32_t arb = fr.can_id & CAN_EFF_MASK;
            char hex[17] = {};
            int dlc = (fr.can_dlc <= 8) ? fr.can_dlc : 8;
            for (int i = 0; i < dlc; i++)
                std::snprintf(hex + i * 2, 3, "%02x", fr.data[i]);
            frames_arr.push_back(json{
                {"device_id", get_device_id(arb)},
                {"func_id",   get_func_id(arb)},
                {"dlc",       dlc},
                {"data",      hex}
            });
        }
        return json{{"type","SNIFF_RESULT"},{"id",id},{"status","OK"},
                    {"frames",frames_arr}}.dump();
    }

    if (type == "SEND_NMT") {
        std::string channel  = req.value("channel", "");
        int         device_id = req.value("device_id", -1);
        int         mode      = req.value("mode", -1);

        if (!bus_mgr_->is_open(channel))
            return error("bus not open: " + channel);
        if (device_id < 0 || device_id > 127 || mode < 0 || mode > 255)
            return error("invalid device_id or mode");

        can_frame f{};
        f.can_id  = make_arb_id(FUNC_NMT, static_cast<uint8_t>(device_id));
        f.can_dlc = 2;
        f.data[0] = static_cast<uint8_t>(mode);
        f.data[1] = static_cast<uint8_t>(device_id);
        bus_mgr_->send(channel, f);
        return ack();
    }

    if (type == "SEND_FLASH_STORE") {
        std::string channel  = req.value("channel", "");
        int         device_id = req.value("device_id", -1);

        if (!bus_mgr_->is_open(channel))
            return error("bus not open: " + channel);
        if (device_id < 0 || device_id > 127)
            return error("invalid device_id");

        can_frame f{};
        f.can_id  = make_arb_id(FUNC_FLASH, static_cast<uint8_t>(device_id));
        f.can_dlc = 1;
        f.data[0] = 0x01;
        bus_mgr_->send(channel, f);
        // Flash write takes ~150 ms; wait 300 ms to ensure completion.
        std::this_thread::sleep_for(ms(300));
        return ack();
    }

    if (type == "FEED_WATCHDOG") {
        std::string channel  = req.value("channel", "");
        int         device_id = req.value("device_id", -1);

        if (!bus_mgr_->is_open(channel))
            return error("bus not open: " + channel);
        if (device_id < 0 || device_id > 127)
            return error("invalid device_id");

        can_frame f{};
        f.can_id  = make_arb_id(FUNC_HEARTBEAT, static_cast<uint8_t>(device_id));
        f.can_dlc = 8;
        std::memset(f.data, 0, 8);
        bus_mgr_->send(channel, f);
        return ack();
    }

    if (type == "CALIBRATE_DEVICE") {
        std::string channel  = req.value("channel", "");
        int         device_id = req.value("device_id", -1);
        int         timeout_ms = req.value("timeout_ms", 90000);

        if (!bus_mgr_->is_open(channel))
            return error("bus not open: " + channel);
        if (device_id < 0 || device_id > 127)
            return error("invalid device_id");

        // Enter calibration mode.
        {
            can_frame nmt{};
            nmt.can_id  = make_arb_id(FUNC_NMT, static_cast<uint8_t>(device_id));
            nmt.can_dlc = 2;
            nmt.data[0] = MODE_CALIBRATION;
            nmt.data[1] = static_cast<uint8_t>(device_id);
            bus_mgr_->send(channel, nmt);
        }

        // Poll heartbeats until mode returns to IDLE or DISABLED.
        auto deadline = Clock::now() + ms(timeout_ms);
        while (Clock::now() < deadline) {
            int remaining = static_cast<int>(
                std::chrono::duration_cast<ms>(deadline - Clock::now()).count());
            if (remaining <= 0) break;

            auto fut = generic_listener_.expect_once(channel,
                static_cast<uint8_t>(device_id), FUNC_HEARTBEAT);
            auto stat = fut.wait_for(ms(std::min(remaining, 2000)));

            if (stat != std::future_status::ready) continue;

            can_frame hb = fut.get();
            uint8_t  hb_mode = (hb.can_dlc >= 1) ? hb.data[0] : 0xFF;
            uint32_t hb_err  = 0;
            if (hb.can_dlc >= 5) std::memcpy(&hb_err, hb.data + 1, 4);

            if (hb_err & ERROR_CALIBRATION_ERROR) {
                return json{{"type","CALIBRATE_RESULT"},{"id",id},{"status","ERROR"},
                            {"error_code",hb_err}}.dump();
            }
            if (hb_mode == MODE_IDLE || hb_mode == MODE_DISABLED) {
                return json{{"type","CALIBRATE_RESULT"},{"id",id},{"status","OK"},
                            {"error_code",hb_err}}.dump();
            }
        }
        return json{{"type","CALIBRATE_RESULT"},{"id",id},{"status","TIMEOUT"}}.dump();
    }

    return error("unknown command type: " + type);
}
