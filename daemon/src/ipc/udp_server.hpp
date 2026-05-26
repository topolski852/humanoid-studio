#pragma once

// Listens on a UDP port for JSON command messages from the Python backend.
// Delivers each message to a callback on a dedicated receive thread.
// Sends responses back to the source address of each command.

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <netinet/in.h>

class UdpServer {
public:
    using Handler = std::function<std::string(const std::string& json_request)>;

    // listen_port: 9001
    explicit UdpServer(uint16_t listen_port);
    ~UdpServer();

    // Start the receive thread. handler is called for each incoming JSON message;
    // the returned string is sent back to the sender. Must be called before spin().
    void set_handler(Handler handler);

    // Start listening (launches background thread).
    bool start();

    // Signal stop and join the receive thread.
    void stop();

    bool is_running() const { return running_.load(); }

private:
    void recv_loop();

    uint16_t            port_;
    int                 fd_      = -1;
    std::atomic<bool>   running_ = false;
    std::thread         thread_;
    Handler             handler_;
};
