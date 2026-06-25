"""
Single-actuator abstraction for one Recoil motor controller node.

Each Actuator wraps a CANBus + device_id and exposes the firmware's full
parameter space through typed async methods.  The apply_config() method
maps a JointConfig onto the firmware's SDO parameter map without any
artificial sleep between writes.
"""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Callable, Coroutine, Any

from pydantic import BaseModel, computed_field

from .can_bus import CANBus, CANBusError, Function, Mode, ErrorCode, Parameter
from .robot_config import JointConfig

_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class ActuatorError(Exception):
    """Raised when an actuator-level operation fails."""


class ActuatorTimeoutError(ActuatorError):
    """Raised when a command does not complete within the allowed time."""


class ActuatorCalibrationError(ActuatorError):
    """Raised when the calibration sequence reports an error."""


# ---------------------------------------------------------------------------
# State snapshot
# ---------------------------------------------------------------------------

class ActuatorState(BaseModel):
    """Snapshot of a single actuator's real-time state."""
    position: float = 0.0          # rad (output-side, after gear ratio)
    velocity: float = 0.0          # rad/s
    torque: float = 0.0            # Nm (estimated from Iq * Kt)
    current: float = 0.0           # A  (Iq — quadrature current)
    mode: int = Mode.DISABLED
    mode_name: str = "DISABLED"
    error: int = 0
    bus_voltage: float | None = None  # V; None = SDO read failed / no data yet
    firmware_version: str | None = None  # "v3.0.8" format; None = not yet read
    timestamp: float = 0.0         # Unix time of last update

    @property
    def has_error(self) -> bool:
        return self.error != 0

    @computed_field
    @property
    def error_names(self) -> list[str]:
        return [m.name for m in ErrorCode if m != ErrorCode.NO_ERROR and (self.error & m)]


# ---------------------------------------------------------------------------
# Actuator
# ---------------------------------------------------------------------------

class Actuator:
    """
    Controls a single Recoil ESC node over CAN.

    Parameters
    ----------
    bus : CANBus
        Shared transport for the CAN channel this device is on.
    config : JointConfig
        Full per-joint configuration (identity + tuning + calibration data).
    """

    def __init__(self, bus: CANBus, config: JointConfig) -> None:
        self._bus = bus
        self._config = config
        self._state = ActuatorState()
        self._state_lock = asyncio.Lock()

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def device_id(self) -> int:
        return self._config.can_id

    @property
    def name(self) -> str:
        return self._config.joint_name

    @property
    def config(self) -> JointConfig:
        return self._config

    def update_config(self, config: JointConfig) -> None:
        self._config = config

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    async def ping(self, timeout: float = 0.1) -> bool:
        """Return True if the device responds to a PDO1 echo."""
        return await self._bus.ping(self.device_id, timeout=timeout)

    async def connect(self, timeout: float = 1.0) -> bool:
        """
        Verify the device is reachable.  Retries up to 3 times.
        Returns True on success.
        """
        for attempt in range(3):
            if await self.ping(timeout=timeout / 3):
                _log.info("Actuator %s (id=%d) connected", self.name, self.device_id)
                return True
            await asyncio.sleep(0.05)
        _log.warning("Actuator %s (id=%d) did not respond", self.name, self.device_id)
        return False

    # ------------------------------------------------------------------
    # Mode control
    # ------------------------------------------------------------------

    async def enable(self, mode: Mode = Mode.POSITION) -> None:
        """Transition to an active control mode."""
        if mode == Mode.POSITION:
            # Read current calibrated position before switching modes.
            # Without an immediate PDO2 after the NMT command, the firmware's
            # position_target stays at 0.0 raw (from PositionController_reset),
            # which maps to calibrated (0.0 - position_offset). With a nonzero
            # position_offset this can be far from the actual joint angle, causing
            # the motor to snap there and whine the moment POSITION mode is entered.
            hold_pos = self._state.position
            if hold_pos is None:
                # No telemetry cached yet — fall back to a live SDO read.
                state = await self.get_state()
                hold_pos = state.position if state.position is not None else 0.0

        await self._bus.set_mode(self.device_id, mode)

        if mode == Mode.POSITION:
            # Fire-and-forget PDO2 with current position. Both this frame and the
            # preceding NMT are queued back-to-back before the event loop yields,
            # so the firmware receives them within one CAN frame period (~200 µs).
            # hold_pos is in display-frame; firmware expects raw (display + offset).
            await self._bus.send_pdo2(
                self.device_id, hold_pos + self._config.position_offset, 0.0
            )

        _log.debug("Actuator %s enabled in mode %s", self.name, mode.name)

    async def disable(self) -> None:
        """Transition to IDLE (PWM off, safe)."""
        await self._bus.set_mode(self.device_id, Mode.IDLE)

    async def damp(self) -> None:
        """Transition to DAMPING (regenerative braking)."""
        await self._bus.set_mode(self.device_id, Mode.DAMPING)

    async def estop(self) -> None:
        """Hard stop — transition to DISABLED (PWM fully off, motor coasts).
        Recovery requires firmware fallback to IDLE + error clear before re-enabling."""
        await self._bus.set_mode(self.device_id, Mode.DISABLED)

    async def feed_watchdog(self) -> None:
        """Reset the 1000 ms safety watchdog timer."""
        await self._bus.feed_watchdog(self.device_id)

    # ------------------------------------------------------------------
    # Parameter read helpers
    # ------------------------------------------------------------------

    async def read_mode(self) -> Mode:
        val = await self._bus.read_parameter_u32(self.device_id, Parameter.MODE)
        if val is None:
            raise ActuatorTimeoutError(f"{self.name}: MODE read timed out")
        try:
            return Mode(val)
        except ValueError:
            return Mode.DISABLED

    async def read_error(self) -> int:
        val = await self._bus.read_parameter_u32(self.device_id, Parameter.ERROR)
        return val if val is not None else 0

    async def clear_error(self) -> None:
        """Write 0 to the ERROR register via SDO, clearing all fault bits."""
        await self._bus.write_parameter_u32(self.device_id, Parameter.ERROR, 0)

    async def read_firmware_version(self) -> str:
        val = await self._bus.read_parameter_u32(self.device_id, Parameter.FIRMWARE_VERSION)
        return hex(val) if val is not None else "unknown"

    async def read_bus_voltage(self) -> float | None:
        return await self._bus.read_parameter_f32(
            self.device_id, Parameter.POWERSTAGE_BUS_VOLTAGE_MEASURED
        )

    async def read_flux_offset(self) -> float | None:
        return await self._bus.read_parameter_f32(
            self.device_id, Parameter.ENCODER_FLUX_OFFSET
        )

    # ------------------------------------------------------------------
    # State snapshot
    # ------------------------------------------------------------------

    async def get_state(
        self,
        passive: tuple[float, float] | None = None,
    ) -> ActuatorState:
        """
        Read position, velocity, torque, mode, error, and bus voltage.

        If passive=(pos_raw_rad, vel_rads) is supplied (from CanMonitor passive sniffing),
        the position and velocity SDO reads are skipped, reducing CAN traffic from 7 to 5
        reads per call.  pos_raw_rad must be in the same frame as
        POSITION_CONTROLLER_POSITION_MEASURED (output-shaft, gear_ratio applied,
        position_offset NOT subtracted).
        """
        async with self._state_lock:
            prev = self._state

        if passive is not None:
            pos_raw, vel_raw = passive[0], passive[1]
        else:
            pos_raw  = await self._bus.read_parameter_f32(self.device_id, Parameter.POSITION_CONTROLLER_POSITION_MEASURED)
            vel_raw  = await self._bus.read_parameter_f32(self.device_id, Parameter.POSITION_CONTROLLER_VELOCITY_MEASURED)
        trq_raw  = await self._bus.read_parameter_f32(self.device_id, Parameter.POSITION_CONTROLLER_TORQUE_MEASURED)
        iq_raw   = await self._bus.read_parameter_f32(self.device_id, Parameter.CURRENT_CONTROLLER_I_Q_MEASURED)
        mode_raw = await self._bus.read_parameter_u32(self.device_id, Parameter.MODE)
        err_raw  = await self._bus.read_parameter_u32(self.device_id, Parameter.ERROR)
        vbus_raw = await self._bus.read_parameter_f32(self.device_id, Parameter.POWERSTAGE_BUS_VOLTAGE_MEASURED)

        # Kinematic fields: freeze at last known value on SDO timeout
        # Subtract position_offset to match PositionController_getPositionMeasured() in firmware,
        # which returns (position_measured - position_offset). The raw SDO field stores position_measured
        # without the offset, but gear_ratio is already divided in — so only offset needs correcting here.
        _pos_offset = self._config.position_offset
        pos = (pos_raw - _pos_offset) if pos_raw is not None else prev.position
        # POSITION_CONTROLLER_VELOCITY_MEASURED (0x054) is already output-shaft rad/s —
        # the firmware divides Encoder_getVelocity() by gear_ratio in MotorController_update().
        vel = vel_raw if vel_raw is not None else prev.velocity
        trq = trq_raw if trq_raw is not None else prev.torque
        iq  = iq_raw  if iq_raw  is not None else prev.current
        err = err_raw if err_raw is not None else prev.error

        # bus_voltage: propagate None so the frontend shows '—' instead of 0
        vbus = vbus_raw

        # Mode: freeze at last known value on timeout OR unrecognized integer
        if mode_raw is None:
            mode_val  = prev.mode
            mode_name = prev.mode_name
        else:
            try:
                mode_val  = mode_raw
                mode_name = Mode(mode_raw).name
            except ValueError:
                mode_val  = prev.mode
                mode_name = prev.mode_name

        state = ActuatorState(
            position=pos,
            velocity=vel,
            torque=trq,
            current=iq,
            mode=mode_val,
            mode_name=mode_name,
            error=err,
            bus_voltage=vbus,
            timestamp=time.time(),
        )
        async with self._state_lock:
            self._state = state
        return state

    @property
    def last_state(self) -> ActuatorState:
        """Return the most recently cached state (non-async)."""
        return self._state

    # ------------------------------------------------------------------
    # Real-time control
    # ------------------------------------------------------------------

    async def set_position(
        self,
        position: float,
        velocity_ff: float = 0.0,
        torque_ff: float = 0.0,
        timeout: float = 0.005,
    ) -> tuple[float, float] | None:
        """
        Send a position target via PDO2 and return (position_measured, velocity_measured).
        Also resets the watchdog timer.

        If torque_ff is non-zero it is written to TORQUE_TARGET via SDO before
        the PDO2 command (the firmware adds it as feed-forward in MODE_POSITION).

        NOTE: velocity_ff is included in PDO2 bytes 4–7 per the Recoil protocol spec,
        but the firmware MODE_POSITION control law computes velocity_error as
        (0 - vel_measured) and never reads the velocity_target field.  Passing a
        non-zero velocity_ff has no effect in POSITION mode.
        """
        if torque_ff != 0.0:
            await self._bus.write_parameter_f32(
                self.device_id,
                Parameter.POSITION_CONTROLLER_TORQUE_TARGET,
                torque_ff,
            )

        result = await self._bus.send_recv_pdo2(
            self.device_id, position + self._config.position_offset, velocity_ff, timeout=timeout
        )

        if result is not None:
            pos, vel = result
            async with self._state_lock:
                # Store display-frame position (consistent with get_state())
                self._state.position = pos - self._config.position_offset
                self._state.velocity = vel
                self._state.timestamp = time.time()

        return result

    async def set_velocity(self, velocity: float) -> None:
        """Write a velocity target (MODE_VELOCITY)."""
        await self._bus.write_parameter_f32(
            self.device_id, Parameter.POSITION_CONTROLLER_VELOCITY_TARGET, velocity
        )

    async def set_torque(self, torque: float) -> None:
        """Write a torque target (MODE_TORQUE or feed-forward in MODE_POSITION)."""
        await self._bus.write_parameter_f32(
            self.device_id, Parameter.POSITION_CONTROLLER_TORQUE_TARGET, torque
        )

    async def set_current(self, i_q: float, i_d: float = 0.0) -> None:
        """Write d/q current targets (MODE_CURRENT)."""
        await self._bus.write_parameter_f32(
            self.device_id, Parameter.CURRENT_CONTROLLER_I_Q_TARGET, i_q
        )
        await self._bus.write_parameter_f32(
            self.device_id, Parameter.CURRENT_CONTROLLER_I_D_TARGET, i_d
        )

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    async def calibrate_offset(
        self,
        timeout: float = 90.0,
        on_progress: Callable[[str], None] | None = None,
    ) -> float:
        """
        Trigger and await the encoder flux-offset calibration sequence.

        The firmware runs the full sequence autonomously:
          ramp voltage → sweep forward → sweep backward → compute offset → store Flash

        Returns the measured flux_offset (rad) on success.
        Raises ActuatorCalibrationError on firmware-reported failure.
        Raises ActuatorTimeoutError if the sequence does not finish in `timeout` seconds.
        """
        def _log_progress(msg: str) -> None:
            _log.info("Calibration [%s]: %s", self.name, msg)
            if on_progress:
                on_progress(msg)

        # ── Pre-calibration connectivity check (diagnostic only) ─────────────
        # Use 2 s timeouts to give a freshly-booted ESC time to respond.
        # This is NEVER a hard failure — we always attempt the NMT command
        # regardless, matching the original committed behaviour.  The NMT is
        # what triggers the firmware calibration sequence; blocking before it
        # is sent means the motor never starts.  The poll loop's
        # _MAX_CONSECUTIVE_TIMEOUTS limit (45 s) catches a truly offline device.
        _log_progress("checking device connectivity…")
        vbus = await self._bus.read_parameter_f32(
            self.device_id, Parameter.POWERSTAGE_BUS_VOLTAGE_MEASURED, timeout=2.0
        )
        if vbus is not None:
            _log_progress(f"ESC online — bus voltage = {vbus:.1f} V")
        else:
            mode_raw = await self._bus.read_parameter_u32(
                self.device_id, Parameter.MODE, timeout=2.0
            )
            if mode_raw is not None:
                _log_progress(f"bus voltage unavailable; ESC online (mode=0x{mode_raw:02X})")
            else:
                # No SDO response — log a warning but proceed to send the NMT.
                # If the device is truly offline the poll loop will catch it.
                _log_progress(
                    "WARNING: no SDO response from device — check CAN cable, "
                    "ESC power (12 V motor bus required), and CAN ID. "
                    "Attempting NMT calibration command anyway…"
                )

        # ── Send NMT MODE_CALIBRATION ─────────────────────────────────────────
        # Firmware >= 0x20250226 sends a HEARTBEAT ACK within ~50 ms of the NMT.
        # Pre-register the waiter BEFORE transmitting to avoid the race where the
        # ACK arrives before receive() can register.  Fall back to SDO polling if
        # no HEARTBEAT arrives (older firmware or transient drop).
        _log_progress("sending NMT MODE_CALIBRATION")
        hb_future = await self._bus.pre_receive(
            filter_device_id=self.device_id,
            filter_func=int(Function.HEARTBEAT),
        )
        await self._bus.set_mode(self.device_id, Mode.CALIBRATION)

        # ── Confirm mode via HEARTBEAT ACK (< 100 ms) or SDO fallback (500 ms) ──
        try:
            hb = await asyncio.wait_for(hb_future, timeout=1.5)
            if len(hb.data) >= 1:
                confirmed_mode = Mode(hb.data[0])
                if confirmed_mode == Mode.CALIBRATION:
                    _log_progress("firmware confirmed MODE_CALIBRATION — sweep started")
                elif confirmed_mode == Mode.IDLE:
                    # setMode returned early (e.g. invalid transition) — retry once
                    _log_progress("HEARTBEAT shows IDLE after NMT — retrying")
                    await self._bus.set_mode(self.device_id, Mode.CALIBRATION)
                else:
                    _log_progress(f"unexpected mode after NMT: {confirmed_mode.name}")
            else:
                _log_progress("HEARTBEAT received but no mode byte — proceeding")
        except asyncio.TimeoutError:
            # No HEARTBEAT — older firmware or frame lost; fall back to SDO poll
            await self._bus.cancel_pre_receive(hb_future)
            _log_progress("no HEARTBEAT response — falling back to SDO mode poll")
            await asyncio.sleep(0.5)
            try:
                confirmed_mode = await self.read_mode()
                if confirmed_mode == Mode.IDLE:
                    _log_progress("mode still IDLE after 500 ms — retrying NMT CALIBRATION")
                    await self._bus.set_mode(self.device_id, Mode.CALIBRATION)
                    await asyncio.sleep(0.5)
                elif confirmed_mode == Mode.CALIBRATION:
                    _log_progress("firmware confirmed MODE_CALIBRATION — sweep started")
            except ActuatorTimeoutError:
                _log_progress("no SDO response after NMT — firmware may have started sweep")

        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        t_start = loop.time()
        poll_interval = 1.0
        last_log_t = t_start
        consecutive_timeouts = 0
        # 45 consecutive SDO timeouts (~45 s) before declaring device offline.
        # High pole-pair motors (14–21 poles) take 12–17 s to calibrate; SDO reads
        # often timeout throughout that window because the firmware main loop is
        # blocked in HAL_Delay during the sweep.  45 s gives enough headroom for
        # the sweep to finish and mode to return to IDLE before we give up.
        _MAX_CONSECUTIVE_TIMEOUTS = 45

        while loop.time() < deadline:
            await asyncio.sleep(poll_interval)
            try:
                mode = await self.read_mode()
            except ActuatorTimeoutError:
                consecutive_timeouts += 1
                if consecutive_timeouts >= _MAX_CONSECUTIVE_TIMEOUTS:
                    raise ActuatorCalibrationError(
                        f"{self.name}: device not responding after {consecutive_timeouts} "
                        f"consecutive SDO timeouts — check CAN connection and ESC power"
                    )
                # Log once on first timeout, then every 5 to avoid log spam
                if consecutive_timeouts == 1 or consecutive_timeouts % 5 == 0:
                    _log_progress(
                        f"no SDO response ({consecutive_timeouts}/{_MAX_CONSECUTIVE_TIMEOUTS}) "
                        f"— device may still be calibrating"
                    )
                continue

            consecutive_timeouts = 0
            elapsed = loop.time() - t_start

            if mode == Mode.IDLE:
                err = await self.read_error()
                flux_offset = await self.read_flux_offset()
                if flux_offset is None:
                    raise ActuatorCalibrationError(
                        f"{self.name}: could not read flux offset after calibration (error=0x{err:04X})"
                    )
                if err & ErrorCode.CALIBRATION_ERROR:
                    # Firmware set the flag but still computed an offset; treat as soft warning
                    _log_progress(
                        f"WARNING: CALIBRATION_ERROR flag set (0x{err:04X}) but "
                        f"flux_offset = {flux_offset:.4f} — using measured value"
                    )

                _log_progress(f"calibration complete — flux_offset = {flux_offset:.4f} rad")
                self._config = self._config.with_electrical_offset(flux_offset)
                return flux_offset

            elif mode == Mode.DISABLED:
                err = await self.read_error()
                if err & ErrorCode.CALIBRATION_ERROR:
                    raise ActuatorCalibrationError(
                        f"{self.name}: firmware reported CALIBRATION_ERROR (error=0x{err:04X})"
                    )
                # Transient DISABLED without CALIBRATION_ERROR — continue polling
                _log_progress(f"transient DISABLED (error=0x{err:04X}) — continuing to poll")

            now = loop.time()
            if now - last_log_t >= 5.0:
                _log_progress(f"still calibrating — {elapsed:.0f}s elapsed (mode={mode.name})")
                last_log_t = now

        raise ActuatorTimeoutError(
            f"{self.name}: calibration did not complete within {timeout:.0f} s"
        )

    # ------------------------------------------------------------------
    # Configuration apply / read-back
    # ------------------------------------------------------------------

    async def apply_config(self) -> None:
        """
        Write all JointConfig fields to the device RAM via SDO.
        No delays between writes — SDO is synchronous request-response.
        """
        c = self._config
        d = self.device_id
        b = self._bus

        # Float32 parameters
        f32: list[tuple[Parameter, float]] = [
            (Parameter.POSITION_CONTROLLER_GEAR_RATIO,            c.gear_ratio),
            (Parameter.POSITION_CONTROLLER_POSITION_KP,           c.position_kp),
            (Parameter.POSITION_CONTROLLER_POSITION_KI,           c.position_ki),
            (Parameter.POSITION_CONTROLLER_VELOCITY_KP,           c.velocity_kp),
            (Parameter.POSITION_CONTROLLER_VELOCITY_KI,           c.velocity_ki),
            (Parameter.POSITION_CONTROLLER_TORQUE_LIMIT,          c.torque_limit),
            (Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT,        c.velocity_limit),
            (Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER,  c.position_limits.lower_bound + c.position_offset),
            (Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER,  c.position_limits.upper_bound + c.position_offset),
            (Parameter.POSITION_CONTROLLER_POSITION_OFFSET,       c.position_offset),
            (Parameter.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA,   c.torque_filter_alpha),
            (Parameter.CURRENT_CONTROLLER_I_LIMIT,                c.current_limit),
            (Parameter.CURRENT_CONTROLLER_I_KP,                   c.current_kp),
            (Parameter.CURRENT_CONTROLLER_I_KI,                   c.current_ki),
            (Parameter.POWERSTAGE_UNDERVOLTAGE_THRESHOLD,         c.undervoltage_threshold),
            (Parameter.POWERSTAGE_OVERVOLTAGE_THRESHOLD,          c.overvoltage_threshold),
            (Parameter.POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA,       c.bus_voltage_filter_alpha),
            (Parameter.MOTOR_TORQUE_CONSTANT,                     c.torque_constant),
            (Parameter.MOTOR_MAX_CALIBRATION_CURRENT,             c.max_calibration_current),
            (Parameter.ENCODER_POSITION_OFFSET,                   c.encoder_position_offset),
            (Parameter.ENCODER_VELOCITY_FILTER_ALPHA,             c.velocity_filter_alpha),
            (Parameter.ENCODER_FLUX_OFFSET,                       c.electrical_offset),
        ]

        # Unsigned 32-bit parameters
        u32: list[tuple[Parameter, int]] = [
            (Parameter.FAST_FRAME_FREQUENCY, c.fast_frame_frequency),
            (Parameter.WATCHDOG_TIMEOUT,     c.watchdog_timeout),
            (Parameter.MOTOR_POLE_PAIRS,     c.pole_pairs),
            (Parameter.ENCODER_CPR,          c.cpr),
        ]

        # Signed 32-bit parameters
        i32: list[tuple[Parameter, int]] = [
            (Parameter.MOTOR_PHASE_ORDER, c.phase_order),
        ]

        for param, val in f32:
            await b.write_parameter_f32(d, param, float(val))
        for param, val in u32:
            await b.write_parameter_u32(d, param, int(val))
        for param, val in i32:
            await b.write_parameter_i32(d, param, int(val))

        _log.info("Config applied to %s (id=%d)", self.name, d)

    async def read_calibration_params(self) -> dict:
        """Read only the three calibration fields from device RAM.
        Returns None for any field that times out — callers must not overwrite
        existing config values with None."""
        d, b = self.device_id, self._bus
        return {
            "electrical_offset":       await b.read_parameter_f32(d, Parameter.ENCODER_FLUX_OFFSET),
            "encoder_position_offset": await b.read_parameter_f32(d, Parameter.ENCODER_POSITION_OFFSET),
            "position_offset":         await b.read_parameter_f32(d, Parameter.POSITION_CONTROLLER_POSITION_OFFSET),
        }

    async def read_config_from_device(self) -> dict:
        """Read all tunable parameters back from device RAM."""
        d = self.device_id
        b = self._bus

        async def rf(p: Parameter) -> float:
            return await b.read_parameter_f32(d, p) or 0.0

        async def ru(p: Parameter) -> int:
            return await b.read_parameter_u32(d, p) or 0

        async def ri(p: Parameter) -> int:
            return await b.read_parameter_i32(d, p) or 0

        return {
            "device_id":              await ru(Parameter.DEVICE_ID),
            "firmware_version":       hex(await ru(Parameter.FIRMWARE_VERSION)),
            "watchdog_timeout":       await ru(Parameter.WATCHDOG_TIMEOUT),
            "fast_frame_frequency":   await ru(Parameter.FAST_FRAME_FREQUENCY),
            "gear_ratio":             await rf(Parameter.POSITION_CONTROLLER_GEAR_RATIO),
            "position_kp":            await rf(Parameter.POSITION_CONTROLLER_POSITION_KP),
            "position_ki":            await rf(Parameter.POSITION_CONTROLLER_POSITION_KI),
            "velocity_kp":            await rf(Parameter.POSITION_CONTROLLER_VELOCITY_KP),
            "velocity_ki":            await rf(Parameter.POSITION_CONTROLLER_VELOCITY_KI),
            "torque_limit":           await rf(Parameter.POSITION_CONTROLLER_TORQUE_LIMIT),
            "velocity_limit":         await rf(Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT),
            "position_limit_lower":   await rf(Parameter.POSITION_CONTROLLER_POSITION_LIMIT_LOWER),
            "position_limit_upper":   await rf(Parameter.POSITION_CONTROLLER_POSITION_LIMIT_UPPER),
            "position_offset":        await rf(Parameter.POSITION_CONTROLLER_POSITION_OFFSET),
            "torque_filter_alpha":    await rf(Parameter.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA),
            "current_limit":          await rf(Parameter.CURRENT_CONTROLLER_I_LIMIT),
            "current_kp":             await rf(Parameter.CURRENT_CONTROLLER_I_KP),
            "current_ki":             await rf(Parameter.CURRENT_CONTROLLER_I_KI),
            "undervoltage_threshold": await rf(Parameter.POWERSTAGE_UNDERVOLTAGE_THRESHOLD),
            "overvoltage_threshold":  await rf(Parameter.POWERSTAGE_OVERVOLTAGE_THRESHOLD),
            "bus_voltage_filter_alpha": await rf(Parameter.POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA),
            "pole_pairs":             await ru(Parameter.MOTOR_POLE_PAIRS),
            "torque_constant":        await rf(Parameter.MOTOR_TORQUE_CONSTANT),
            "phase_order":            await ri(Parameter.MOTOR_PHASE_ORDER),
            "max_calibration_current": await rf(Parameter.MOTOR_MAX_CALIBRATION_CURRENT),
            "cpr":                    await ru(Parameter.ENCODER_CPR),
            "encoder_position_offset": await rf(Parameter.ENCODER_POSITION_OFFSET),
            "velocity_filter_alpha":  await rf(Parameter.ENCODER_VELOCITY_FILTER_ALPHA),
            "electrical_offset":      await rf(Parameter.ENCODER_FLUX_OFFSET),
        }

    # ------------------------------------------------------------------
    # Flash persistence
    # ------------------------------------------------------------------

    async def store_to_flash(self) -> None:
        """Persist current RAM config to firmware Flash page 63."""
        await self._bus.store_to_flash(self.device_id)
        _log.info("Config stored to Flash: %s (id=%d)", self.name, self.device_id)

    async def load_from_flash(self) -> None:
        """Reload Flash config into device RAM."""
        await self._bus.load_from_flash(self.device_id)

    # ------------------------------------------------------------------
    # PID / bandwidth helpers
    # ------------------------------------------------------------------

    async def set_current_bandwidth(
        self, bandwidth_hz: float, phase_resistance: float, phase_inductance: float
    ) -> None:
        """Calculate and write current PI gains from motor parameters."""
        import math
        kp = bandwidth_hz * 2.0 * math.pi * phase_inductance
        ki = phase_resistance / phase_inductance
        await self._bus.write_parameter_f32(self.device_id, Parameter.CURRENT_CONTROLLER_I_KP, kp)
        await self._bus.write_parameter_f32(self.device_id, Parameter.CURRENT_CONTROLLER_I_KI, ki)
        _log.debug("Current bandwidth set: %s → kp=%.4f ki=%.1f", self.name, kp, ki)

    async def set_torque_bandwidth(
        self, bandwidth_hz: float, position_loop_rate: float = 2000.0
    ) -> None:
        """Calculate and write torque filter alpha from bandwidth."""
        import math
        alpha = min(max(1.0 - math.exp(-2.0 * math.pi * bandwidth_hz / position_loop_rate), 0.0), 1.0)
        await self._bus.write_parameter_f32(
            self.device_id, Parameter.POSITION_CONTROLLER_TORQUE_FILTER_ALPHA, alpha
        )

    def __repr__(self) -> str:
        return (
            f"Actuator(name={self.name!r}, "
            f"can_id={self.device_id}, "
            f"channel={self._config.can_channel!r})"
        )
