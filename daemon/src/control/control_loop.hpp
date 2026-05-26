#pragma once

// Real-time loop utility — concept adapted from Berkeley LoopFunc.
// Runs a callable at a fixed period on a dedicated thread with optional
// SCHED_FIFO scheduling and CPU affinity.

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include <pthread.h>
#include <sched.h>

class ControlLoop {
public:
    struct Options {
        double      period_s    = 0.005;  // 200 Hz default
        int         sched_prio  = 0;      // 0 = SCHED_OTHER; > 0 = SCHED_FIFO
        int         cpu_affinity = -1;    // -1 = no affinity
        std::string name;
    };

    ControlLoop() = default;

    // Start the loop thread. callable is invoked once per period.
    void start(std::function<void()> callable, const Options& opts) {
        callable_ = std::move(callable);
        opts_     = opts;
        running_  = true;
        thread_   = std::thread(&ControlLoop::loop_body, this);

        // Apply CPU affinity and scheduling after thread creation.
        if (opts_.cpu_affinity >= 0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(opts_.cpu_affinity, &cpuset);
            pthread_setaffinity_np(thread_.native_handle(), sizeof(cpuset), &cpuset);
        }

        if (opts_.sched_prio > 0) {
            struct sched_param sp{ .sched_priority = opts_.sched_prio };
            int rc = pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sp);
            if (rc != 0) {
                fprintf(stderr, "[ControlLoop:%s] SCHED_FIFO failed (rc=%d) — "
                        "run with cap_sys_nice or as root\n",
                        opts_.name.c_str(), rc);
            }
        }
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    bool is_running() const { return running_.load(); }

    uint64_t overrun_count() const { return overruns_.load(); }

private:
    void loop_body() {
        using Clock     = std::chrono::steady_clock;
        using Duration  = std::chrono::duration<double>;
        using Ns        = std::chrono::nanoseconds;

        const Ns period = std::chrono::duration_cast<Ns>(Duration(opts_.period_s));
        const Ns overrun_warn = period / 2;  // warn if overrun > 50% of period

        auto next_wake = Clock::now() + period;

        while (running_.load()) {
            callable_();

            auto now = Clock::now();
            auto remaining = next_wake - now;
            if (remaining.count() < 0) {
                if (-remaining > overrun_warn) {
                    ++overruns_;
                    fprintf(stderr, "[ControlLoop:%s] overrun by %lld µs\n",
                            opts_.name.c_str(),
                            static_cast<long long>(-remaining.count() / 1000));
                }
                // Catch up: skip to the next period boundary.
                next_wake = now + period;
            } else {
                std::this_thread::sleep_for(remaining);
                next_wake += period;
            }
        }
    }

    std::function<void()>  callable_;
    Options                opts_;
    std::atomic<bool>      running_ = false;
    std::atomic<uint64_t>  overruns_ = 0;
    std::thread            thread_;
};
