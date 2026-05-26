#include "can/socket_can.hpp"

#include <cstdio>
#include <cstring>
#include <cerrno>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// Berkeley uses blocking select() + ::read() on one socket.
// We use O_NONBLOCK + ::read() so the control loop never stalls
// regardless of how many buses are open.

SocketCan::SocketCan(const std::string& ifname) : ifname_(ifname) {}

SocketCan::~SocketCan() {
    close();
}

bool SocketCan::open() {
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        fprintf(stderr, "[SocketCan] socket() failed for %s: %s\n",
                ifname_.c_str(), strerror(errno));
        return false;
    }

    // Resolve interface name → index (same approach as Berkeley socketcan.cpp)
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "[SocketCan] ioctl SIOCGIFINDEX failed for %s: %s\n",
                ifname_.c_str(), strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Set non-blocking — key difference from Berkeley's blocking reads
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        fprintf(stderr, "[SocketCan] fcntl O_NONBLOCK failed for %s: %s\n",
                ifname_.c_str(), strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Bind to the interface
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[SocketCan] bind failed for %s: %s\n",
                ifname_.c_str(), strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void SocketCan::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SocketCan::send(const can_frame& frame) {
    ssize_t n = ::write(fd_, &frame, sizeof(frame));
    if (n < 0) {
        if (errno == ENOBUFS || errno == EAGAIN) {
            // TX queue full — caller may retry; log at debug level
            tx_dropped_++;
            return false;
        }
        fprintf(stderr, "[SocketCan] write error on %s: %s\n",
                ifname_.c_str(), strerror(errno));
        return false;
    }
    return true;
}

bool SocketCan::recv(can_frame& frame) {
    ssize_t n = ::read(fd_, &frame, sizeof(frame));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;  // no frame available — normal non-blocking case
        }
        fprintf(stderr, "[SocketCan] read error on %s: %s\n",
                ifname_.c_str(), strerror(errno));
        return false;
    }
    if (n != (ssize_t)sizeof(can_frame)) {
        return false;  // short read — discard
    }
    rx_frames_++;
    return true;
}
