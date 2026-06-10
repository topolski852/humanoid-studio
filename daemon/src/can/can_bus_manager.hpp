#pragma once

// Owns N SocketCan instances keyed by interface name.
// Provides a single send/drain_all surface to the control loop.

#include "can/socket_can.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <linux/can.h>

class CanBusManager {
public:
    // Open one socket per unique name in ifnames. Missing interfaces are logged
    // but non-fatal; is_open(ifname) will return false for them.
    explicit CanBusManager(const std::vector<std::string>& ifnames);
    ~CanBusManager() = default;

    // Send a frame on the named interface. No-op (with log) if bus is not open.
    void send(const std::string& ifname, const can_frame& frame);

    // Drain pending Rx frames across all open buses (up to max_frames total per call).
    // Calls on_frame(ifname, frame) for each frame received.
    // Capped to prevent control-loop overruns when a large backlog accumulates.
    void drain_all(const std::function<void(const std::string&, const can_frame&)>& on_frame,
                   int max_frames = 64);

    bool is_open(const std::string& ifname) const;

    // Aggregate drop counter across all buses.
    uint64_t total_tx_dropped() const;

    // Per-bus access for telemetry.
    struct BusStats {
        bool     open;
        uint64_t tx_dropped;
        uint64_t rx_frames;
    };
    std::unordered_map<std::string, BusStats> stats() const;

private:
    std::unordered_map<std::string, std::unique_ptr<SocketCan>> buses_;
};
