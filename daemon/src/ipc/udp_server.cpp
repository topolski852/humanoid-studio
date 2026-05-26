#include "ipc/udp_server.hpp"

#include <cstdio>
#include <cstring>
#include <cerrno>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>

static constexpr int MAX_DGRAM = 65507;

UdpServer::UdpServer(uint16_t listen_port) : port_(listen_port) {}

UdpServer::~UdpServer() {
    stop();
}

void UdpServer::set_handler(Handler handler) {
    handler_ = std::move(handler);
}

bool UdpServer::start() {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        fprintf(stderr, "[UdpServer] socket() failed: %s\n", strerror(errno));
        return false;
    }

    // SO_REUSEADDR so we can restart the daemon quickly.
    int yes = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    // Non-blocking with a 100 ms receive timeout so the loop can check running_.
    struct timeval tv{ .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port_);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        fprintf(stderr, "[UdpServer] bind on port %u failed: %s\n", port_, strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    running_ = true;
    thread_  = std::thread(&UdpServer::recv_loop, this);
    fprintf(stderr, "[UdpServer] listening on 127.0.0.1:%u\n", port_);
    return true;
}

void UdpServer::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

void UdpServer::recv_loop() {
    std::string buf(MAX_DGRAM, '\0');
    struct sockaddr_in sender{};
    socklen_t sender_len = sizeof(sender);

    while (running_.load()) {
        ssize_t n = recvfrom(fd_, buf.data(), buf.size(), 0,
                             reinterpret_cast<struct sockaddr*>(&sender), &sender_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            if (!running_.load()) break;
            fprintf(stderr, "[UdpServer] recvfrom error: %s\n", strerror(errno));
            continue;
        }

        std::string request(buf.data(), static_cast<size_t>(n));
        std::string response;
        if (handler_) {
            try {
                response = handler_(request);
            } catch (const std::exception& e) {
                response = std::string("{\"type\":\"ERROR\",\"msg\":\"") + e.what() + "\"}";
            }
        }

        if (!response.empty()) {
            sendto(fd_, response.data(), response.size(), 0,
                   reinterpret_cast<const struct sockaddr*>(&sender), sender_len);
        }
    }
}
