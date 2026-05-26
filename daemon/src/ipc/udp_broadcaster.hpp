#pragma once

// Sends JSON telemetry frames to a fixed destination (Python backend).
// Non-blocking: each send() call is fire-and-forget on a UDP socket.

#include <string>
#include <netinet/in.h>

class UdpBroadcaster {
public:
    // dest_addr: "127.0.0.1", dest_port: 9000
    UdpBroadcaster(const std::string& dest_addr, uint16_t dest_port);
    ~UdpBroadcaster();

    // Send a JSON string as one UDP datagram. Returns false on error.
    bool send(const std::string& json);

    bool is_open() const { return fd_ >= 0; }

private:
    int              fd_   = -1;
    struct sockaddr_in dest_{};
};
