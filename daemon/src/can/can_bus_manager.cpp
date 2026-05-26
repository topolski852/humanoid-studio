#include "can/can_bus_manager.hpp"

#include <cstdio>

CanBusManager::CanBusManager(const std::vector<std::string>& ifnames) {
    for (const auto& name : ifnames) {
        if (buses_.count(name)) continue;  // deduplicate
        auto bus = std::make_unique<SocketCan>(name);
        if (!bus->open()) {
            fprintf(stderr, "[CanBusManager] interface %s unavailable — joints on this bus will be OFFLINE\n",
                    name.c_str());
            // Insert with open() already failed; is_open() returns false via fd_<0
        }
        buses_.emplace(name, std::move(bus));
    }
}

void CanBusManager::send(const std::string& ifname, const can_frame& frame) {
    auto it = buses_.find(ifname);
    if (it == buses_.end() || !it->second->is_open()) {
        fprintf(stderr, "[CanBusManager] send on closed bus %s — dropping frame\n", ifname.c_str());
        return;
    }
    it->second->send(frame);
}

void CanBusManager::drain_all(
    const std::function<void(const std::string&, const can_frame&)>& on_frame)
{
    for (auto& [name, bus] : buses_) {
        if (!bus->is_open()) continue;
        can_frame frame;
        while (bus->recv(frame)) {
            on_frame(name, frame);
        }
    }
}

bool CanBusManager::is_open(const std::string& ifname) const {
    auto it = buses_.find(ifname);
    return it != buses_.end() && it->second->is_open();
}

uint64_t CanBusManager::total_tx_dropped() const {
    uint64_t total = 0;
    for (auto& [name, bus] : buses_) {
        total += bus->tx_dropped();
    }
    return total;
}

std::unordered_map<std::string, CanBusManager::BusStats> CanBusManager::stats() const {
    std::unordered_map<std::string, BusStats> out;
    for (auto& [name, bus] : buses_) {
        out[name] = BusStats{
            bus->is_open(),
            bus->tx_dropped(),
            bus->rx_frames()
        };
    }
    return out;
}
