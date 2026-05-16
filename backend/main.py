"""
FastAPI backend for Humanoid Studio.

Starts on http://localhost:8765
WebSocket /ws/telemetry broadcasts:
  - Motor telemetry at 10 Hz:   {"connected": bool, "actuators": {...}}
  - CAN health every 200 ms:    {"type": "can_health", "interfaces": [...]}
  - CAN drop events (immediate): {"type": "can_drop_event", ...}
"""
from __future__ import annotations

import asyncio
import logging
import math
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from humanoid.robot_config import RobotConfig
from humanoid.robot import Robot
from humanoid.flash import FlashManager
from humanoid.can_monitor import CanMonitor
from humanoid import settings as _app_settings
from api import devices_router, motors_router, flash_router, robot_router, debug_router, settings_router

logging.basicConfig(level=logging.INFO)
_log = logging.getLogger(__name__)
_TELEMETRY_HZ   = 20
_HEALTH_EVERY_N = 4    # emit can_health every 200 ms (4 × 50 ms motor frames)

_VEL_ALPHA = 0.2    # EMA weight on newest velocity sample; at 20 Hz → ~225 ms time constant
_smooth: dict[str, dict] = {}  # per-joint EMA state, persists across WS reconnects

_WATCHDOG_FEED_HZ = 5   # background heartbeat rate; 200 ms interval, 5× margin vs 1000 ms timeout


async def _watchdog_task(app: FastAPI) -> None:
    """Feed safety watchdog on every connected ESC, regardless of WS clients."""
    interval = 1.0 / _WATCHDOG_FEED_HZ
    while True:
        await asyncio.sleep(interval)
        robot: Robot | None = app.state.robot
        if robot and robot.is_connected():
            try:
                await robot.feed_all_watchdogs()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Application lifespan
# ---------------------------------------------------------------------------

@asynccontextmanager
async def lifespan(app: FastAPI):
    config_file = Path(_app_settings.load()["config_path"])
    config: RobotConfig | None = None
    if config_file.exists():
        try:
            config = RobotConfig.from_json(config_file)
            _log.info("Loaded robot config: %s (%d joints)", config.robot_name, len(config.joints))
        except Exception as exc:
            _log.warning("Failed to load config from %s: %s", config_file, exc)

    app.state.config      = config
    app.state.config_path = config_file
    app.state.robot       = Robot(config) if config else None
    app.state.flash_manager = FlashManager()
    app.state.ws_clients: set[WebSocket] = set()

    # CAN monitor — starts background polling/sniffing tasks
    monitor = CanMonitor(config)
    app.state.can_monitor = monitor
    await monitor.start()

    # Background watchdog feed — keeps all active ESCs alive even when no WS client is connected
    wdt_task = asyncio.create_task(_watchdog_task(app))

    yield

    wdt_task.cancel()
    try:
        await wdt_task
    except asyncio.CancelledError:
        pass

    robot: Robot | None = app.state.robot
    if robot and robot.is_connected():
        await robot.disconnect()

    await monitor.stop()


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

app = FastAPI(title="Humanoid Studio", version="0.1.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(devices_router)
app.include_router(motors_router)
app.include_router(flash_router)
app.include_router(robot_router)
app.include_router(debug_router)
app.include_router(settings_router)


# ---------------------------------------------------------------------------
# WebSocket telemetry
# ---------------------------------------------------------------------------

@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket) -> None:
    await ws.accept()
    app.state.ws_clients.add(ws)
    _log.info("Telemetry client connected (total=%d)", len(app.state.ws_clients))

    interval      = 1.0 / _TELEMETRY_HZ
    health_counter = _HEALTH_EVERY_N - 1  # emit health on first tick so UI gets CAN status immediately
    monitor: CanMonitor = app.state.can_monitor

    try:
        while True:
            robot: Robot | None = app.state.robot

            if robot is None or not robot.is_connected():
                actuator_payload = {"connected": False, "actuators": {}}
                online_joints: set[str] = set()
            else:
                states = await robot.get_all_states()
                await robot.feed_all_watchdogs()
                online_joints = {n for n, s in states.items() if s is not None}
                actuator_dicts: dict = {}
                for name, s in states.items():
                    if s is None:
                        actuator_dicts[name] = None
                        continue
                    d = s.model_dump()
                    d["error_names"] = s.error_names()
                    sm = _smooth.setdefault(name, {})
                    raw_v = d["velocity"] if d["velocity"] is not None and math.isfinite(d["velocity"]) else sm.get("velocity", 0.0)
                    sm["velocity"] = _VEL_ALPHA * raw_v + (1 - _VEL_ALPHA) * sm.get("velocity", raw_v)
                    d["velocity"] = sm["velocity"]
                    # bus_voltage: no EMA — pass through as float or None; frontend shows '—' for None
                    actuator_dicts[name] = d
                actuator_payload = {"connected": True, "actuators": actuator_dicts}

            # Send motor telemetry frame
            try:
                await ws.send_json(actuator_payload)
            except Exception:
                break

            # Drain and forward any CAN drop events immediately
            for evt in await monitor.pop_drop_events():
                try:
                    await ws.send_json(evt)
                except Exception:
                    break

            # Every 2 s: send CAN health snapshot
            # (joints_online is now derived from passive traffic in get_interface_stats)
            health_counter += 1
            if health_counter >= _HEALTH_EVERY_N:
                health_counter = 0
                try:
                    await ws.send_json({
                        "type": "can_health",
                        "interfaces": monitor.get_interface_stats(),
                    })
                except Exception:
                    break

            await asyncio.sleep(interval)

    except WebSocketDisconnect:
        pass
    except Exception as exc:
        _log.debug("Telemetry WS error: %s", exc)
    finally:
        app.state.ws_clients.discard(ws)
        _log.info("Telemetry client disconnected (total=%d)", len(app.state.ws_clients))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    uvicorn.run("main:app", host="localhost", port=8765, reload=False, log_level="info")
