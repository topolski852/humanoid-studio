#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

#include "config/config_loader.hpp"
#include "control/robot.hpp"

// ── Signal handling ──────────────────────────────────────────────────────────

static std::atomic<int> g_signal_count{0};
static Robot*           g_robot = nullptr;

static void signal_handler(int sig) {
    ++g_signal_count;
    // Second signal: force exit immediately (async-signal-safe).
    if (g_signal_count.load() >= 2) _exit(1);
    (void)sig;
}

// ── Argument parsing ─────────────────────────────────────────────────────────

static void print_usage(const char* argv0) {
    fprintf(stderr,
        "Usage: %s [OPTIONS]\n"
        "Options:\n"
        "  --config PATH      JSON config file (default: ../configs/humanoid_lite.json)\n"
        "  --cmd-port PORT    UDP command port (default: 9001)\n"
        "  --tel-port PORT    UDP telemetry port (default: 9000)\n"
        "  --tel-hz HZ        Telemetry rate 1-100 Hz (default: 10)\n"
        "  --rt-prio PRIO     SCHED_FIFO priority 0-99 (default: 80; 0=SCHED_OTHER)\n"
        "  --cpu CPU          CPU affinity for control loop (default: 0)\n"
        "  --help             Show this message\n",
        argv0);
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    const char* config_path = "../configs/humanoid_lite.json";
    RobotOptions opts;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else if (strcmp(argv[i], "--cmd-port") == 0 && i + 1 < argc) {
            opts.cmd_port = static_cast<uint16_t>(atoi(argv[++i]));
        } else if (strcmp(argv[i], "--tel-port") == 0 && i + 1 < argc) {
            opts.telemetry_port = static_cast<uint16_t>(atoi(argv[++i]));
        } else if (strcmp(argv[i], "--tel-hz") == 0 && i + 1 < argc) {
            opts.telemetry_hz = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--rt-prio") == 0 && i + 1 < argc) {
            opts.control_prio = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--cpu") == 0 && i + 1 < argc) {
            opts.control_cpu = atoi(argv[++i]);
        } else {
            fprintf(stderr, "[daemon] unknown argument: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    fprintf(stderr, "[daemon] config:      %s\n", config_path);
    fprintf(stderr, "[daemon] cmd-port:    %u\n", opts.cmd_port);
    fprintf(stderr, "[daemon] tel-port:    %u\n", opts.telemetry_port);
    fprintf(stderr, "[daemon] tel-hz:      %d\n", opts.telemetry_hz);
    fprintf(stderr, "[daemon] rt-prio:     %d\n", opts.control_prio);
    fprintf(stderr, "[daemon] cpu:         %d\n", opts.control_cpu);

    // Load config.
    RobotConfig cfg;
    try {
        cfg = load_config(config_path);
    } catch (const std::exception& e) {
        fprintf(stderr, "[daemon] config load failed: %s\n", e.what());
        return 1;
    }
    fprintf(stderr, "[daemon] loaded %zu joints from %s\n",
            cfg.joints.size(), config_path);

    // Override telemetry_hz from config if not overridden on CLI.
    if (opts.telemetry_hz == 10 && cfg.telemetry_hz > 0)
        opts.telemetry_hz = cfg.telemetry_hz;

    // Install signal handlers.
    struct sigaction sa{};
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // Start daemon.
    Robot robot(std::move(cfg), opts);
    g_robot = &robot;

    if (!robot.start()) {
        fprintf(stderr, "[daemon] startup failed\n");
        return 1;
    }
    fprintf(stderr, "[daemon] running — send SIGINT or UDP SHUTDOWN to stop\n");

    // Block until first signal.
    while (g_signal_count.load() == 0) {
        pause();  // sleep until signal arrives
    }

    fprintf(stderr, "[daemon] stopping…\n");
    robot.stop();
    fprintf(stderr, "[daemon] exited cleanly\n");
    return 0;
}
