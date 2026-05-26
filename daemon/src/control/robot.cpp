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
    // 1. Drain all pending Rx frames and dispatch to actuators.
    bus_mgr_->drain_all([this](const std::string& ifname, const can_frame& frame) {
        (void)ifname;
        uint32_t arb = frame.can_id & CAN_EFF_MASK;
        int dev_id = static_cast<int>(get_device_id(arb));
        for (auto& a : actuators_) {
            if (a->device_id() == dev_id && a->can_channel() == ifname) {
                a->on_rx_frame(frame);
                break;
            }
        }
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

    return error("unknown command type: " + type);
}
