#pragma once

// Single SocketCAN interface wrapper.
// Non-blocking I/O: recv() returns false immediately if no frame is available.
// Approach informed by Berkeley socketcan.cpp; written fresh for our design.

#include <string>
#include <cstdint>

#include <linux/can.h>
#include <net/if.h>

class SocketCan {
public:
    explicit SocketCan(const std::string& ifname);
    ~SocketCan();

    // Open and bind the socket. Returns true on success.
    // Pass silent=true to suppress error logs (used for background reconnect attempts).
    bool open(bool silent = false);
    void close();
    bool is_open() const { return fd_ >= 0; }
    const std::string& ifname() const { return ifname_; }

    // Send one frame. Returns false if the TX queue is full (ENOBUFS).
    bool send(const can_frame& frame);

    // Receive one frame without blocking.
    // Returns true and fills frame on success; returns false if no frame available.
    bool recv(can_frame& frame);

    // Statistics
    uint64_t tx_dropped() const { return tx_dropped_; }
    uint64_t rx_frames()  const { return rx_frames_; }

private:
    std::string ifname_;
    int         fd_         = -1;
    uint64_t    tx_dropped_ = 0;
    uint64_t    rx_frames_  = 0;
};
