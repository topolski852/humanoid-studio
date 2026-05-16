"""
Robot — top-level coordinator that opens CAN buses and owns all Actuator objects.
"""
from __future__ import annotations

import asyncio
from pathlib import Path
from typing import AsyncIterator

import logging

from .can_bus import CANBus, CANBusError, Mode

_log = logging.getLogger(__name__)
from .actuator import Actuator, ActuatorState
from .robot_config import RobotConfig, JointConfig


class Robot:
    """
    Opens one CANBus per unique channel in the config, creates one Actuator
    per joint, and provides multi-joint coordination helpers.
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        self._buses: dict[str, CANBus] = {}
        self._actuators: dict[str, Actuator] = {}          # joint_name → Actuator
        self._actuators_by_id: dict[int, Actuator] = {}    # can_id → Actuator (first win)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def connect(self) -> None:
        # Connect each CAN bus independently — missing adapters are logged and skipped.
        # Joints on unavailable buses simply won't appear in telemetry (offline).
        for channel in self._config.channels():
            bus = CANBus(channel=channel)
            try:
                await bus.connect()
                self._buses[channel] = bus
                _log.info("CAN bus connected: %s", channel)
            except CANBusError as exc:
                _log.warning(
                    "CAN bus %s unavailable — joints on this bus will be offline: %s",
                    channel, exc,
                )

        for joint_name, joint_cfg in self._config:
            bus = self._buses.get(joint_cfg.can_channel)
            if bus is None:
                _log.debug("Skipping %s — bus %s not connected", joint_name, joint_cfg.can_channel)
                continue
            actuator = Actuator(bus, joint_cfg)
            self._actuators[joint_name] = actuator
            self._actuators_by_id.setdefault(joint_cfg.can_id, actuator)

    async def disconnect(self) -> None:
        for bus in self._buses.values():
            await bus.disconnect()
        self._buses.clear()
        self._actuators.clear()
        self._actuators_by_id.clear()

    async def __aenter__(self) -> "Robot":
        await self.connect()
        return self

    async def __aexit__(self, *_: object) -> None:
        await self.disconnect()

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def config(self) -> RobotConfig:
        return self._config

    def get_actuator(self, joint_name: str) -> Actuator:
        return self._actuators[joint_name]

    def get_actuator_by_name(self, joint_name: str) -> Actuator | None:
        return self._actuators.get(joint_name)

    def get_actuator_by_id(self, can_id: int) -> Actuator | None:
        return self._actuators_by_id.get(can_id)

    def joint_names(self) -> list[str]:
        return list(self._actuators.keys())

    def actuators(self) -> list[Actuator]:
        return list(self._actuators.values())

    def is_connected(self) -> bool:
        return bool(self._buses)

    # ------------------------------------------------------------------
    # Multi-joint operations
    # ------------------------------------------------------------------

    async def ping_all(self, timeout: float = 0.5) -> dict[str, bool]:
        results = await asyncio.gather(
            *[a.ping(timeout=timeout) for a in self._actuators.values()],
            return_exceptions=True,
        )
        return {
            name: (result is True)
            for name, result in zip(self._actuators.keys(), results)
        }

    async def connect_all(self, timeout: float = 1.0) -> dict[str, bool]:
        results = await asyncio.gather(
            *[a.connect(timeout=timeout) for a in self._actuators.values()],
            return_exceptions=True,
        )
        return {
            name: (result is True)
            for name, result in zip(self._actuators.keys(), results)
        }

    async def enable_all(self, mode: Mode = Mode.POSITION) -> None:
        await asyncio.gather(*[a.enable(mode) for a in self._actuators.values()])

    async def disable_all(self) -> None:
        await asyncio.gather(*[a.disable() for a in self._actuators.values()])

    async def damp_all(self) -> None:
        await asyncio.gather(*[a.damp() for a in self._actuators.values()])

    async def get_all_states(self) -> dict[str, ActuatorState | None]:
        async def _safe_state(name: str, actuator: Actuator) -> tuple[str, ActuatorState | None]:
            try:
                return name, await actuator.get_state()
            except Exception:
                return name, None

        pairs = await asyncio.gather(
            *[_safe_state(n, a) for n, a in self._actuators.items()]
        )
        return dict(pairs)

    async def feed_all_watchdogs(self) -> None:
        await asyncio.gather(
            *[a.feed_watchdog() for a in self._actuators.values()],
            return_exceptions=True,
        )

    async def read_calibration_from_devices(self) -> None:
        """Pull hardware-calibrated values (flux_offset, encoder offsets) from each ESC
        into the in-memory config so a subsequent apply_config() won't zero them out.
        Runs sequentially to avoid TX queue bursts on a busy CAN bus."""
        for actuator in self._actuators.values():
            try:
                raw = await actuator.read_calibration_params()
                # Only overwrite if the SDO read actually returned a non-None value;
                # a None/timed-out read must not replace valid calibration data with 0.
                if raw.get("electrical_offset") is not None:
                    actuator.config.electrical_offset = raw["electrical_offset"]
                if raw.get("encoder_position_offset") is not None:
                    actuator.config.encoder_position_offset = raw["encoder_position_offset"]
                if raw.get("position_offset") is not None:
                    actuator.config.position_offset = raw["position_offset"]
            except Exception as exc:
                _log.warning("Calibration pull from %s failed: %s", actuator.name, exc)

    async def apply_all_configs(self) -> None:
        """Write design params to each ESC sequentially to avoid TX queue bursts."""
        for actuator in self._actuators.values():
            try:
                await actuator.apply_config()
            except Exception as exc:
                _log.warning("apply_config failed for %s: %s", actuator.name, exc)

    async def store_all_to_flash(self) -> None:
        await asyncio.gather(*[a.store_to_flash() for a in self._actuators.values()])
