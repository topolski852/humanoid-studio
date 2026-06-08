"""
Async CAN bus transport for the Recoil motor controller protocol.

Design:
- A single background asyncio task reads from the SocketCAN socket in a
  thread executor and dispatches frames to registered waiters.
- Multiple coroutines can await frames concurrently; each gets its own Future
  that is resolved by the first matching frame (FIFO among waiters).
- The transmit path holds an asyncio.Lock so concurrent sends never interleave.
"""
from __future__ import annotations

import asyncio
import logging
import struct
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Sequence

import can

_log = logging.getLogger(__name__)

DEVICE_ID_MASK = 0x7F
FUNC_ID_SHIFT = 7


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class CANBusError(Exception):
    """Raised for CAN transport-level errors."""


# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

class Function(IntEnum):
    NMT               = 0x0
    SYNC_EMCY         = 0x1
    TIME              = 0x2
    TRANSMIT_PDO_1    = 0x3
    RECEIVE_PDO_1     = 0x4
    TRANSMIT_PDO_2    = 0x5
    RECEIVE_PDO_2     = 0x6
    TRANSMIT_PDO_3    = 0x7
    RECEIVE_PDO_3     = 0x8
    TRANSMIT_PDO_4    = 0x9
    RECEIVE_PDO_4     = 0xA
    TRANSMIT_SDO      = 0xB
    RECEIVE_SDO       = 0xC
    FLASH             = 0xD
    HEARTBEAT         = 0xE


class Mode(IntEnum):
    DISABLED            = 0x00
    IDLE                = 0x01
    DAMPING             = 0x02
    CALIBRATION         = 0x05
    CURRENT             = 0x10
    TORQUE              = 0x11
    VELOCITY            = 0x12
    POSITION            = 0x13
    VABC_OVERRIDE       = 0x20
    VALPHABETA_OVERRIDE = 0x21
    VQD_OVERRIDE        = 0x22
    DEBUG               = 0x80


class ErrorCode(IntEnum):
    NO_ERROR             = 0b0000_0000_0000_0000
    GENERAL              = 0b0000_0000_0000_0001
    ESTOP                = 0b0000_0000_0000_0010
    INITIALIZATION_ERROR = 0b0000_0000_0000_0100
    CALIBRATION_ERROR    = 0b0000_0000_0000_1000
    POWERSTAGE_ERROR     = 0b0000_0000_0001_0000
    INVALID_MODE         = 0b0000_0000_0010_0000
    WATCHDOG_TIMEOUT     = 0b0000_0000_0100_0000
    OVER_VOLTAGE         = 0b0000_0000_1000_0000
    OVER_CURRENT         = 0b0000_0001_0000_0000
    OVER_TEMPERATURE     = 0b0000_0010_0000_0000
    CAN_RX_FAULT         = 0b0000_0100_0000_0000
    CAN_TX_FAULT         = 0b0000_1000_0000_0000
    I2C_FAULT            = 0b0001_0000_0000_0000
    ENCODER_FAULT        = 0b0010_0000_0000_0000


class Parameter(IntEnum):
    """Byte offsets into the firmware MotorController struct (SDO address map)."""
    DEVICE_ID                                       = 0x000
    FIRMWARE_VERSION                                = 0x004
    WATCHDOG_TIMEOUT                                = 0x008
    FAST_FRAME_FREQUENCY                            = 0x00C
    MODE                                            = 0x010
    ERROR                                           = 0x014
    POSITION_CONTROLLER_UPDATE_COUNTER              = 0x018
    POSITION_CONTROLLER_GEAR_RATIO                  = 0x01C
    POSITION_CONTROLLER_POSITION_KP                 = 0x020
    POSITION_CONTROLLER_POSITION_KI                 = 0x024
    POSITION_CONTROLLER_VELOCITY_KP                 = 0x028
    POSITION_CONTROLLER_VELOCITY_KI                 = 0x02C
    POSITION_CONTROLLER_TORQUE_LIMIT                = 0x030
    POSITION_CONTROLLER_VELOCITY_LIMIT              = 0x034
    POSITION_CONTROLLER_POSITION_LIMIT_LOWER        = 0x038
    POSITION_CONTROLLER_POSITION_LIMIT_UPPER        = 0x03C
    POSITION_CONTROLLER_POSITION_OFFSET             = 0x040
    POSITION_CONTROLLER_TORQUE_TARGET               = 0x044
    POSITION_CONTROLLER_TORQUE_MEASURED             = 0x048
    POSITION_CONTROLLER_TORQUE_SETPOINT             = 0x04C
    POSITION_CONTROLLER_VELOCITY_TARGET             = 0x050
    POSITION_CONTROLLER_VELOCITY_MEASURED           = 0x054
    POSITION_CONTROLLER_VELOCITY_SETPOINT           = 0x058
    POSITION_CONTROLLER_POSITION_TARGET             = 0x05C
    POSITION_CONTROLLER_POSITION_MEASURED           = 0x060
    POSITION_CONTROLLER_POSITION_SETPOINT           = 0x064
    POSITION_CONTROLLER_POSITION_INTEGRATOR         = 0x068
    POSITION_CONTROLLER_VELOCITY_INTEGRATOR         = 0x06C
    POSITION_CONTROLLER_TORQUE_FILTER_ALPHA         = 0x070
    CURRENT_CONTROLLER_I_LIMIT                      = 0x074
    CURRENT_CONTROLLER_I_KP                         = 0x078
    CURRENT_CONTROLLER_I_KI                         = 0x07C
    CURRENT_CONTROLLER_I_A_MEASURED                 = 0x080
    CURRENT_CONTROLLER_I_B_MEASURED                 = 0x084
    CURRENT_CONTROLLER_I_C_MEASURED                 = 0x088
    CURRENT_CONTROLLER_V_A_SETPOINT                 = 0x08C
    CURRENT_CONTROLLER_V_B_SETPOINT                 = 0x090
    CURRENT_CONTROLLER_V_C_SETPOINT                 = 0x094
    CURRENT_CONTROLLER_I_ALPHA_MEASURED             = 0x098
    CURRENT_CONTROLLER_I_BETA_MEASURED              = 0x09C
    CURRENT_CONTROLLER_V_ALPHA_SETPOINT             = 0x0A0
    CURRENT_CONTROLLER_V_BETA_SETPOINT              = 0x0A4
    CURRENT_CONTROLLER_V_Q_TARGET                   = 0x0A8
    CURRENT_CONTROLLER_V_D_TARGET                   = 0x0AC
    CURRENT_CONTROLLER_V_Q_SETPOINT                 = 0x0B0
    CURRENT_CONTROLLER_V_D_SETPOINT                 = 0x0B4
    CURRENT_CONTROLLER_I_Q_TARGET                   = 0x0B8
    CURRENT_CONTROLLER_I_D_TARGET                   = 0x0BC
    CURRENT_CONTROLLER_I_Q_MEASURED                 = 0x0C0
    CURRENT_CONTROLLER_I_D_MEASURED                 = 0x0C4
    CURRENT_CONTROLLER_I_Q_SETPOINT                 = 0x0C8
    CURRENT_CONTROLLER_I_D_SETPOINT                 = 0x0CC
    CURRENT_CONTROLLER_I_Q_INTEGRATOR               = 0x0D0
    CURRENT_CONTROLLER_I_D_INTEGRATOR               = 0x0D4
    POWERSTAGE_HTIM                                 = 0x0D8
    POWERSTAGE_HADC1                                = 0x0DC
    POWERSTAGE_HADC2                                = 0x0E0
    POWERSTAGE_ADC_READING_RAW                      = 0x0E4
    POWERSTAGE_ADC_READING_OFFSET                   = 0x0EC
    POWERSTAGE_UNDERVOLTAGE_THRESHOLD               = 0x0F4
    POWERSTAGE_OVERVOLTAGE_THRESHOLD                = 0x0F8
    POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA             = 0x0FC
    POWERSTAGE_BUS_VOLTAGE_MEASURED                 = 0x100
    MOTOR_POLE_PAIRS                                = 0x104
    MOTOR_TORQUE_CONSTANT                           = 0x108
    MOTOR_PHASE_ORDER                               = 0x10C
    MOTOR_MAX_CALIBRATION_CURRENT                   = 0x110
    ENCODER_HI2C                                    = 0x114
    ENCODER_I2C_BUFFER                              = 0x118
    ENCODER_I2C_UPDATE_COUNTER                      = 0x11C
    ENCODER_CPR                                     = 0x120
    ENCODER_POSITION_OFFSET                         = 0x124
    ENCODER_VELOCITY_FILTER_ALPHA                   = 0x128
    ENCODER_POSITION_RAW                            = 0x12C
    ENCODER_N_ROTATIONS                             = 0x130
    ENCODER_POSITION                                = 0x134
    ENCODER_VELOCITY                                = 0x138
    ENCODER_FLUX_OFFSET                             = 0x13C
    ENCODER_FLUX_OFFSET_TABLE                       = 0x140

    # Virtual parameters — not struct offsets; handled specially by the firmware SDO handler
    SAVE_CONFIG                                     = 0xFF00  # write non-zero → storeConfig()
    REBOOT                                          = 0xFF04  # write non-zero → NVIC_SystemReset()


# SDO command specifiers (bits [7:5] of the command byte)
_SDO_CCS_WRITE = 0x01 << 5   # 0x20  — download (host → node)
_SDO_CCS_READ  = 0x02 << 5   # 0x40  — upload  (node → host)


# ---------------------------------------------------------------------------
# Frame
# ---------------------------------------------------------------------------

@dataclass
class CANFrame:
    device_id: int
    func_id: int
    data: bytes = field(default_factory=bytes)

    @property
    def size(self) -> int:
        return len(self.data)

    def __repr__(self) -> str:
        hex_data = self.data.hex(" ") if self.data else "(empty)"
        return (
            f"CANFrame(dev={self.device_id}, "
            f"func={Function(self.func_id).name if self.func_id in Function._value2member_map_ else self.func_id:#x}, "
            f"data=[{hex_data}])"
        )


# ---------------------------------------------------------------------------
# Internal waiter
# ---------------------------------------------------------------------------

@dataclass
class _Waiter:
    device_id: int | None
    func_id: int | None
    future: asyncio.Future

    def matches(self, frame: CANFrame) -> bool:
        if self.device_id is not None and frame.device_id != self.device_id:
            return False
        if self.func_id is not None and frame.func_id != self.func_id:
            return False
        return True


# ---------------------------------------------------------------------------
# CANBus
# ---------------------------------------------------------------------------

class CANBus:
    """
    Async SocketCAN interface. Supports multiple concurrent receive waiters.

    Usage::

        async with CANBus("can0") as bus:
            ok = await bus.ping(device_id=1)
    """

    def __init__(self, channel: str, bitrate: int = 1_000_000):
        self.channel = channel
        self.bitrate = bitrate

        self._raw_bus: can.interface.Bus | None = None
        self._running = False
        self._receive_task: asyncio.Task | None = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"can_{channel}")
        self._waiters: list[_Waiter] = []
        self._waiters_lock = asyncio.Lock()
        self._tx_lock = asyncio.Lock()
        # Per-device SDO lock: guarantees only one SDO transaction is in flight per
        # device_id at a time.  Firmware now echoes param_id in 8-byte responses, so
        # concurrent reads would no longer silently corrupt each other, but the lock
        # is kept for serialization correctness.
        self._device_sdo_locks: dict[int, asyncio.Lock] = {}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def connect(self) -> None:
        if self._raw_bus is not None:
            return
        try:
            self._raw_bus = can.interface.Bus(
                interface="socketcan",
                channel=self.channel,
                bitrate=self.bitrate,
            )
        except Exception as exc:
            raise CANBusError(f"Failed to open {self.channel}: {exc}") from exc

        self._running = True
        self._receive_task = asyncio.create_task(
            self._receive_loop(), name=f"can_rx_{self.channel}"
        )
        _log.info("CANBus connected: %s", self.channel)

    async def disconnect(self) -> None:
        self._running = False
        if self._receive_task is not None:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass
            self._receive_task = None

        if self._raw_bus is not None:
            self._raw_bus.shutdown()
            self._raw_bus = None

        # Fail all pending waiters
        async with self._waiters_lock:
            for w in self._waiters:
                if not w.future.done():
                    w.future.cancel()
            self._waiters.clear()

        self._executor.shutdown(wait=True)
        _log.info("CANBus disconnected: %s", self.channel)

    async def __aenter__(self) -> "CANBus":
        await self.connect()
        return self

    async def __aexit__(self, *_: object) -> None:
        await self.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._raw_bus is not None

    @property
    def is_available(self) -> bool:
        """True if the underlying adapter is present and connected."""
        return self._raw_bus is not None

    # ------------------------------------------------------------------
    # Receive loop (background task)
    # ------------------------------------------------------------------

    def _blocking_recv(self) -> can.Message | None:
        """Called in executor; blocks ≤ 10 ms."""
        try:
            return self._raw_bus.recv(timeout=0.01)  # type: ignore[union-attr]
        except Exception as exc:
            _log.debug("CAN recv exception on %s: %s", self.channel, exc)
            return None

    async def _receive_loop(self) -> None:
        loop = asyncio.get_running_loop()
        while self._running:
            try:
                msg = await loop.run_in_executor(self._executor, self._blocking_recv)
            except asyncio.CancelledError:
                break
            except Exception as exc:
                if self._running:
                    _log.error("CAN recv error on %s: %s", self.channel, exc)
                continue

            if msg is None:
                continue
            if msg.is_error_frame:
                _log.warning("Error frame on %s: arb_id=0x%03X", self.channel, msg.arbitration_id)
                continue

            frame = CANFrame(
                device_id=msg.arbitration_id & DEVICE_ID_MASK,
                func_id=msg.arbitration_id >> FUNC_ID_SHIFT,
                data=bytes(msg.data),
            )
            await self._dispatch(frame)

    async def _dispatch(self, frame: CANFrame) -> None:
        async with self._waiters_lock:
            for i, waiter in enumerate(self._waiters):
                if waiter.matches(frame) and not waiter.future.done():
                    waiter.future.set_result(frame)
                    self._waiters.pop(i)
                    return
        # Frame had no waiter — log at debug level only
        _log.debug("Unmatched frame: %s", frame)

    # ------------------------------------------------------------------
    # Transmit
    # ------------------------------------------------------------------

    async def transmit(self, frame: CANFrame) -> None:
        if self._raw_bus is None:
            raise CANBusError(f"Bus {self.channel} is not connected — adapter offline or not yet plugged in")
        if frame.device_id > DEVICE_ID_MASK:
            raise CANBusError(f"device_id {frame.device_id} exceeds 7-bit range")

        arb_id = (frame.func_id << FUNC_ID_SHIFT) | frame.device_id
        msg = can.Message(arbitration_id=arb_id, is_extended_id=False, data=frame.data)

        async with self._tx_lock:
            for attempt in range(4):
                try:
                    self._raw_bus.send(msg)
                    break
                except can.CanError as exc:
                    # ENOBUFS (errno 105): SocketCAN TX queue full — ESC boot burst or
                    # rapid back-to-back SDO writes overflow the kernel queue.  Retry
                    # with short backoff; raise on final attempt.
                    cause = exc.__cause__ or exc.__context__
                    is_nobufs = (
                        isinstance(cause, OSError) and cause.errno == 105
                    ) or "No buffer space" in str(exc) or "105" in str(exc)
                    if is_nobufs and attempt < 3:
                        await asyncio.sleep(0.05 * (attempt + 1))
                        continue
                    raise CANBusError(f"Transmit failed on {self.channel}: {exc}") from exc
            # Yield to the event loop so the kernel can drain the hardware TX FIFO
            # before the next frame is queued.
            await asyncio.sleep(0)

    # ------------------------------------------------------------------
    # Receive with filter & timeout
    # ------------------------------------------------------------------

    async def pre_receive(
        self,
        filter_device_id: int | None = None,
        filter_func: int | None = None,
    ) -> "asyncio.Future[CANFrame]":
        """Register a receive waiter and return its Future WITHOUT blocking.

        Call this BEFORE transmitting the frame that triggers a response, then
        ``await asyncio.wait_for(future, timeout=...)`` after transmitting.
        This avoids the race where a fast firmware response arrives before
        a subsequent ``receive()`` call can register its waiter.
        """
        loop = asyncio.get_running_loop()
        future: asyncio.Future[CANFrame] = loop.create_future()
        waiter = _Waiter(filter_device_id, filter_func, future)
        async with self._waiters_lock:
            self._waiters.append(waiter)
        return future

    async def cancel_pre_receive(self, future: "asyncio.Future[CANFrame]") -> None:
        """Remove a pre_receive future that was never consumed."""
        async with self._waiters_lock:
            self._waiters[:] = [w for w in self._waiters if w.future is not future]
        if not future.done():
            future.cancel()

    async def receive(
        self,
        filter_device_id: int | None = None,
        filter_func: int | None = None,
        timeout: float | None = None,
    ) -> CANFrame | None:
        """
        Wait for a frame matching the given filters.

        Registers a waiter BEFORE returning so no frame is missed between
        a transmit() call and this receive() call.
        """
        loop = asyncio.get_running_loop()
        future: asyncio.Future[CANFrame] = loop.create_future()
        waiter = _Waiter(filter_device_id, filter_func, future)

        async with self._waiters_lock:
            self._waiters.append(waiter)

        try:
            return await asyncio.wait_for(future, timeout=timeout)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            async with self._waiters_lock:
                try:
                    self._waiters.remove(waiter)
                except ValueError:
                    pass
            return None

    # ------------------------------------------------------------------
    # SDO primitives
    # ------------------------------------------------------------------

    def _sdo_lock(self, device_id: int) -> asyncio.Lock:
        """Return (creating if needed) the per-device SDO serialization lock."""
        if device_id not in self._device_sdo_locks:
            self._device_sdo_locks[device_id] = asyncio.Lock()
        return self._device_sdo_locks[device_id]

    async def _sdo_read(
        self, device_id: int, param_id: int, timeout: float = 0.1
    ) -> bytes | None:
        """Send SDO read request; return 4 raw value bytes or None on timeout.

        Standard 8-byte CANopen upload response: byte0=0x43, bytes1-2=param_id echo,
        byte3=0, bytes4-7=value.  The per-device lock serializes transactions.
        """
        async with self._sdo_lock(device_id):
            # Register waiter BEFORE transmitting — avoids race if response arrives fast
            loop = asyncio.get_running_loop()
            future: asyncio.Future[CANFrame] = loop.create_future()
            waiter = _Waiter(device_id, Function.TRANSMIT_SDO, future)
            async with self._waiters_lock:
                self._waiters.append(waiter)

            request = CANFrame(
                device_id=device_id,
                func_id=Function.RECEIVE_SDO,
                # data[0]=command, data[1:3]=param_id, data[3:8]=zeros (DLC must be 8)
                data=struct.pack("<BHB4x", _SDO_CCS_READ, param_id, 0),
            )
            await self.transmit(request)

            try:
                rx = await asyncio.wait_for(future, timeout=timeout)
                # Standard 8-byte response: byte0=0x43, bytes1-2=param_id, bytes4-7=value
                return bytes(rx.data[4:8])
            except (asyncio.TimeoutError, asyncio.CancelledError):
                async with self._waiters_lock:
                    try:
                        self._waiters.remove(waiter)
                    except ValueError:
                        pass
                _log.debug("SDO read timeout: dev=%d param=0x%03X", device_id, param_id)
                return None

    async def _sdo_write(self, device_id: int, param_id: int, raw: bytes) -> None:
        """Send SDO write and drain the firmware write ACK.

        Acquires the per-device SDO lock so writes do not interleave with
        concurrent reads (which would cause the read waiter to miss its response).

        Firmware sends a standard 8-byte CANopen download response (byte0=0x60, bytes1-2=
        param_id echo) on FUNC_TRANSMIT_SDO after each write.  We register a waiter
        BEFORE transmitting so the ACK is consumed here and never leaks to a concurrent
        _sdo_read() waiter.  The 15 ms timeout covers the sub-millisecond CAN round-trip.
        """
        assert len(raw) == 4, "SDO value must be exactly 4 bytes"
        async with self._sdo_lock(device_id):
            loop = asyncio.get_running_loop()
            ack_future: asyncio.Future[CANFrame] = loop.create_future()
            ack_waiter = _Waiter(device_id, Function.TRANSMIT_SDO, ack_future)
            async with self._waiters_lock:
                self._waiters.append(ack_waiter)

            frame = CANFrame(
                device_id=device_id,
                func_id=Function.RECEIVE_SDO,
                data=struct.pack("<BHB", _SDO_CCS_WRITE, param_id, 0) + raw,
            )
            await self.transmit(frame)

            try:
                await asyncio.wait_for(ack_future, timeout=0.015)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                async with self._waiters_lock:
                    try:
                        self._waiters.remove(ack_waiter)
                    except ValueError:
                        pass

    # ------------------------------------------------------------------
    # Typed SDO helpers
    # ------------------------------------------------------------------

    async def read_parameter_f32(
        self, device_id: int, param_id: int, timeout: float = 0.025
    ) -> float | None:
        raw = await self._sdo_read(device_id, param_id, timeout)
        if raw is None:
            return None
        (val,) = struct.unpack("<f", raw)
        return val

    async def read_parameter_i32(
        self, device_id: int, param_id: int, timeout: float = 0.025
    ) -> int | None:
        raw = await self._sdo_read(device_id, param_id, timeout)
        if raw is None:
            return None
        (val,) = struct.unpack("<l", raw)
        return val

    async def read_parameter_u32(
        self, device_id: int, param_id: int, timeout: float = 0.025
    ) -> int | None:
        raw = await self._sdo_read(device_id, param_id, timeout)
        if raw is None:
            return None
        (val,) = struct.unpack("<L", raw)
        return val

    async def write_parameter_f32(
        self, device_id: int, param_id: int, value: float
    ) -> None:
        await self._sdo_write(device_id, param_id, struct.pack("<f", value))

    async def write_parameter_i32(
        self, device_id: int, param_id: int, value: int
    ) -> None:
        await self._sdo_write(device_id, param_id, struct.pack("<l", value))

    async def write_parameter_u32(
        self, device_id: int, param_id: int, value: int
    ) -> None:
        await self._sdo_write(device_id, param_id, struct.pack("<L", value))

    # ------------------------------------------------------------------
    # High-level protocol helpers
    # ------------------------------------------------------------------

    async def ping(self, device_id: int, timeout: float = 0.1) -> bool:
        """Send PDO1 echo and verify the magic byte is reflected."""
        frame = CANFrame(
            device_id=device_id,
            func_id=Function.RECEIVE_PDO_1,
            data=b"\xCA",
        )
        loop = asyncio.get_running_loop()
        future: asyncio.Future[CANFrame] = loop.create_future()
        waiter = _Waiter(device_id, Function.TRANSMIT_PDO_1, future)
        async with self._waiters_lock:
            self._waiters.append(waiter)

        await self.transmit(frame)

        try:
            rx = await asyncio.wait_for(future, timeout=timeout)
            return len(rx.data) >= 1 and rx.data[0] == 0xCA
        except (asyncio.TimeoutError, asyncio.CancelledError):
            async with self._waiters_lock:
                try:
                    self._waiters.remove(waiter)
                except ValueError:
                    pass
            return False

    async def feed_watchdog(self, device_id: int) -> None:
        """Send HEARTBEAT to reset the firmware watchdog timer."""
        await self.transmit(CANFrame(device_id, Function.HEARTBEAT, b""))

    async def set_mode(self, device_id: int, mode: Mode) -> None:
        """Send NMT frame to change the operating mode."""
        await self.transmit(CANFrame(
            device_id=device_id,
            func_id=Function.NMT,
            data=struct.pack("<BB", int(mode), device_id),
        ))

    async def store_to_flash(self, device_id: int) -> None:
        """Persist current RAM config to Flash page 63."""
        await self.transmit(CANFrame(
            device_id=device_id,
            func_id=Function.FLASH,
            data=struct.pack("<B", 1),
        ))

    async def load_from_flash(self, device_id: int) -> None:
        """Reload Flash config into RAM."""
        await self.transmit(CANFrame(
            device_id=device_id,
            func_id=Function.FLASH,
            data=struct.pack("<B", 2),
        ))

    async def send_pdo2(
        self, device_id: int, position_target: float, velocity_ff: float = 0.0
    ) -> None:
        """Send position + velocity_ff targets. Also resets watchdog."""
        await self.transmit(CANFrame(
            device_id=device_id,
            func_id=Function.RECEIVE_PDO_2,
            data=struct.pack("<ff", position_target, velocity_ff),
        ))

    async def recv_pdo2(
        self, device_id: int, timeout: float = 0.005
    ) -> tuple[float, float] | None:
        """Wait for PDO2 response: (position_measured, velocity_measured)."""
        rx = await self.receive(
            filter_device_id=device_id,
            filter_func=Function.TRANSMIT_PDO_2,
            timeout=timeout,
        )
        if rx is None or len(rx.data) < 8:
            return None
        pos, vel = struct.unpack("<ff", rx.data[:8])
        return pos, vel

    async def send_recv_pdo2(
        self,
        device_id: int,
        position_target: float,
        velocity_ff: float = 0.0,
        timeout: float = 0.005,
    ) -> tuple[float, float] | None:
        """Send PDO2 command and wait for the echoed measurements."""
        loop = asyncio.get_running_loop()
        future: asyncio.Future[CANFrame] = loop.create_future()
        waiter = _Waiter(device_id, Function.TRANSMIT_PDO_2, future)
        async with self._waiters_lock:
            self._waiters.append(waiter)

        await self.send_pdo2(device_id, position_target, velocity_ff)

        try:
            rx = await asyncio.wait_for(future, timeout=timeout)
            pos, vel = struct.unpack("<ff", rx.data[:8])
            return pos, vel
        except (asyncio.TimeoutError, asyncio.CancelledError):
            async with self._waiters_lock:
                try:
                    self._waiters.remove(waiter)
                except ValueError:
                    pass
            return None
