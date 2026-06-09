#pragma once

// Thread-safe listener for one-shot and multi-frame CAN capture.
// on_frame() is called from the 200 Hz control thread.
// All other methods are called from the UDP handler thread.

#include <atomic>
#include <cstdint>
#include <future>
#include <mutex>
#include <string>
#include <vector>

#include <linux/can.h>

class GenericListener {
public:
    static constexpr uint8_t WILDCARD = 0xFF;

    // Register a one-shot expectation; returns a future that resolves on
    // the first matching frame. WILDCARD matches any device_id or func_id.
    std::future<can_frame> expect_once(const std::string& channel,
                                       uint8_t device_id, uint8_t func_id);

    // Begin collecting all matching frames. Returns a sniff_id.
    uint64_t begin_sniff(const std::string& channel,
                         uint8_t device_id = WILDCARD,
                         uint8_t func_id   = WILDCARD);

    // Stop collecting and return all frames gathered so far. Removes the entry.
    std::vector<can_frame> end_sniff(uint64_t sniff_id);

    // Called for every incoming frame from drain_all() — control thread.
    void on_frame(const std::string& channel, const can_frame& frame);

    // Remove all unfulfilled one-shots matching (channel, device_id, func_id).
    // Must be called by UDP handlers after wait_for() times out so that stale
    // entries cannot consume ACKs intended for the next registered future.
    // Returns the number of entries removed.
    int cancel_stale(const std::string& channel, uint8_t device_id, uint8_t func_id);

private:
    struct OneShot {
        std::string             channel;
        uint8_t                 device_id;
        uint8_t                 func_id;
        std::promise<can_frame> promise;
        bool                    fulfilled{false};
    };

    struct SniffEntry {
        uint64_t               id;
        std::string            channel;
        uint8_t                device_id;
        uint8_t                func_id;
        std::vector<can_frame> frames;
    };

    bool matches(const std::string& ch, const can_frame& f,
                 const std::string& exp_ch, uint8_t exp_dev, uint8_t exp_func) const;

    std::mutex              lock_;
    std::atomic<int>        pending_{0};  // fast path: skip lock when nothing registered
    uint64_t                next_id_{0};
    std::vector<OneShot>    one_shots_;
    std::vector<SniffEntry> sniffs_;
};
