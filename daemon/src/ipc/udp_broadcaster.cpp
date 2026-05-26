#include "ipc/udp_broadcaster.hpp"

#include <cstdio>
#include <cstring>
#include <cerrno>

#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

UdpBroadcaster::UdpBroadcaster(const std::string& dest_addr, uint16_t dest_port) {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        fprintf(stderr, "[UdpBroadcaster] socket() failed: %s\n", strerror(errno));
        return;
    }
    memset(&dest_, 0, sizeof(dest_));
    dest_.sin_family = AF_INET;
    dest_.sin_port   = htons(dest_port);
    if (inet_pton(AF_INET, dest_addr.c_str(), &dest_.sin_addr) != 1) {
        fprintf(stderr, "[UdpBroadcaster] invalid address: %s\n", dest_addr.c_str());
        ::close(fd_);
        fd_ = -1;
    }
}

UdpBroadcaster::~UdpBroadcaster() {
    if (fd_ >= 0) ::close(fd_);
}

bool UdpBroadcaster::send(const std::string& json) {
    if (fd_ < 0) return false;
    ssize_t n = sendto(fd_, json.data(), json.size(), 0,
                       reinterpret_cast<const struct sockaddr*>(&dest_), sizeof(dest_));
    if (n < 0) {
        fprintf(stderr, "[UdpBroadcaster] sendto failed: %s\n", strerror(errno));
        return false;
    }
    return true;
}
