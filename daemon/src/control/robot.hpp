#pragma once

// Multi-joint coordinator.
// Owns all Actuators, the CanBusManager, the 200 Hz control loop,
// the telemetry broadcast loop, and the UDP command server.
//
// Thread model:
//   - Control thread (200 Hz, SCHED_FIFO):  drain_all → tick all joints → snapshot telemetry
//   - Telemetry thread (10–100 Hz, normal): read telemetry snapshot → broadcast JSON
//   - UDP server thread (normal):           receive commands → enqueue to command_queue_
//   - Main thread:                          start / stop only

#include "config/config_loader.hpp"
#include "can/can_bus_manager.hpp"
#include "can/generic_listener.hpp"
#include "motor/actuator.hpp"
#include "control/control_loop.hpp"
#include "ipc/udp_broadcaster.hpp"
#include "ipc/udp_server.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

struct RobotOptions {
    uint16_t cmd_port       = 9001;
    uint16_t telemetry_port = 9000;
    int      telemetry_hz   = 10;
    int      control_prio   = 80;   // SCHED_FIFO priority for control loop
    int      control_cpu    = 0;    // CPU affinity for control loop
};

class Robot {
public:
    explicit Robot(RobotConfig cfg, RobotOptions opts = RobotOptions{});
    ~Robot();

    // Open all CAN buses, start control loop, telemetry push, and UDP server.
    // Returns false if any critical step fails.
    bool start();

    // Graceful shutdown: damp all joints → idle → stop threads → close sockets.
    void stop();

    bool is_running() const { return running_.load(); }

private:
    // --- Control-loop tick (200 Hz) ---
    void control_tick();

    // --- Telemetry thread ---
    void telemetry_loop();

    // --- Command dispatch (called from UdpServer handler on its receive thread) ---
    std::string handle_command(const std::string& json);

    // --- Telemetry JSON builder ---
    std::string build_telemetry_json(uint64_t seq);

    RobotConfig   cfg_;
    RobotOptions  opts_;

    std::unique_ptr<CanBusManager>  bus_mgr_;
    GenericListener                 generic_listener_;
    std::vector<std::unique_ptr<Actuator>> actuators_;
    std::unordered_map<std::string, Actuator*> actuator_by_name_;

    ControlLoop  control_loop_;
    UdpServer    udp_server_;
    UdpBroadcaster broadcaster_;

    std::atomic<bool> running_ = false;

    // Telemetry thread
    std::thread telemetry_thread_;

    // Sequence counter for telemetry frames.
    uint64_t telemetry_seq_ = 0;
};
