#include "can/generic_listener.hpp"
#include "motor/recoil_protocol.hpp"

#include <algorithm>
#include <linux/can.h>

std::future<can_frame> GenericListener::expect_once(
    const std::string& channel, uint8_t device_id, uint8_t func_id)
{
    std::lock_guard<std::mutex> lk(lock_);
    one_shots_.push_back(OneShot{channel, device_id, func_id, {}, false});
    pending_.fetch_add(1, std::memory_order_relaxed);
    return one_shots_.back().promise.get_future();
}

uint64_t GenericListener::begin_sniff(
    const std::string& channel, uint8_t device_id, uint8_t func_id)
{
    std::lock_guard<std::mutex> lk(lock_);
    uint64_t id = next_id_++;
    sniffs_.push_back(SniffEntry{id, channel, device_id, func_id, {}});
    pending_.fetch_add(1, std::memory_order_relaxed);
    return id;
}

std::vector<can_frame> GenericListener::end_sniff(uint64_t sniff_id) {
    std::lock_guard<std::mutex> lk(lock_);
    std::vector<can_frame> result;
    auto it = std::find_if(sniffs_.begin(), sniffs_.end(),
                           [sniff_id](const SniffEntry& e){ return e.id == sniff_id; });
    if (it != sniffs_.end()) {
        result = std::move(it->frames);
        sniffs_.erase(it);
        pending_.fetch_sub(1, std::memory_order_relaxed);
    }
    return result;
}

int GenericListener::cancel_stale(
    const std::string& channel, uint8_t device_id, uint8_t func_id)
{
    std::lock_guard<std::mutex> lk(lock_);
    int removed = 0;
    auto new_end = std::remove_if(one_shots_.begin(), one_shots_.end(),
        [&](const OneShot& os) -> bool {
            if (!os.fulfilled &&
                os.channel == channel &&
                os.device_id == device_id &&
                os.func_id == func_id) {
                ++removed;
                return true;
            }
            return false;
        });
    one_shots_.erase(new_end, one_shots_.end());
    if (removed > 0)
        pending_.fetch_sub(removed, std::memory_order_relaxed);
    return removed;
}

bool GenericListener::matches(const std::string& ch, const can_frame& f,
                               const std::string& exp_ch, uint8_t exp_dev,
                               uint8_t exp_func) const
{
    if (ch != exp_ch) return false;
    uint32_t arb = f.can_id & CAN_EFF_MASK;
    uint8_t dev  = get_device_id(arb);
    uint8_t func = get_func_id(arb);
    if (exp_dev  != WILDCARD && dev  != exp_dev)  return false;
    if (exp_func != WILDCARD && func != exp_func) return false;
    return true;
}

void GenericListener::on_frame(const std::string& channel, const can_frame& frame) {
    if (pending_.load(std::memory_order_relaxed) == 0) return;

    std::lock_guard<std::mutex> lk(lock_);

    // Fulfill the first matching one-shot.
    for (auto& os : one_shots_) {
        if (os.fulfilled) continue;
        if (matches(channel, frame, os.channel, os.device_id, os.func_id)) {
            os.promise.set_value(frame);
            os.fulfilled = true;
            pending_.fetch_sub(1, std::memory_order_relaxed);
            break;
        }
    }
    one_shots_.erase(
        std::remove_if(one_shots_.begin(), one_shots_.end(),
                       [](const OneShot& os){ return os.fulfilled; }),
        one_shots_.end());

    // Push to all active sniff collectors (capped at 1000 frames each).
    for (auto& s : sniffs_) {
        if (s.frames.size() < 1000 &&
            matches(channel, frame, s.channel, s.device_id, s.func_id)) {
            s.frames.push_back(frame);
        }
    }
}
