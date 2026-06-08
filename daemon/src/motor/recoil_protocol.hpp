#pragma once

// CAN protocol constants for the Recoil motor controller.
// All values sourced from:
//   - firmware: Core/Inc/motor_controller_conf.h
//   - python:   backend/humanoid/can_bus.py
//   - docs:     HANDOFF.md section 2.1

#include <cstdint>
#include <cstring>

// ─── CAN ID layout ────────────────────────────────────────────────────────────
// bits [10:7] = func_id (4 bits)
// bits [6:0]  = device_id (7 bits)
// arb_id = (func_id << 7) | device_id
static constexpr uint32_t DEVICE_ID_MASK = 0x7Fu;
static constexpr uint32_t FUNC_ID_SHIFT  = 7u;

inline uint32_t make_arb_id(uint8_t func_id, uint8_t device_id) {
    return ((uint32_t)func_id << FUNC_ID_SHIFT) | (device_id & DEVICE_ID_MASK);
}
inline uint8_t get_device_id(uint32_t arb_id) {
    return (uint8_t)(arb_id & DEVICE_ID_MASK);
}
inline uint8_t get_func_id(uint32_t arb_id) {
    return (uint8_t)((arb_id >> FUNC_ID_SHIFT) & 0xFu);
}

// ─── Function codes ───────────────────────────────────────────────────────────
// Source: motor_controller_conf.h FrameFunction enum, confirmed against can_bus.py
enum FuncCode : uint8_t {
    FUNC_NMT              = 0x0,  // host→node: mode change
    FUNC_SYNC_EMCY        = 0x1,  // node→host: EMCY error broadcast (4 bytes)
    FUNC_TIME             = 0x2,  // unused
    FUNC_TRANSMIT_PDO_1   = 0x3,  // node→host: ping echo
    FUNC_RECEIVE_PDO_1    = 0x4,  // host→node: ping request
    FUNC_TRANSMIT_PDO_2   = 0x5,  // node→host: [pos_measured f32, vel_measured f32]
    FUNC_RECEIVE_PDO_2    = 0x6,  // host→node: [pos_target f32, vel_ff f32]
    FUNC_TRANSMIT_PDO_3   = 0x7,  // node→host: calibration status frames
    FUNC_RECEIVE_PDO_3    = 0x8,  // host→node: unused
    FUNC_TRANSMIT_PDO_4   = 0x9,  // node→host: autonomous fast-frame [pos f32, vel f32]
    FUNC_RECEIVE_PDO_4    = 0xA,  // host→node: no-op
    FUNC_TRANSMIT_SDO     = 0xB,  // node→host: SDO response (8 bytes, standard CANopen)
    FUNC_RECEIVE_SDO      = 0xC,  // host→node: SDO read/write request
    FUNC_FLASH            = 0xD,  // host→node: store(1) or load(2) Flash
    FUNC_HEARTBEAT        = 0xE,  // host→node: watchdog feed (0 bytes)
};

// ─── Motor modes ──────────────────────────────────────────────────────────────
// Source: motor_controller_conf.h Mode enum, confirmed against can_bus.py
enum MotorMode : uint8_t {
    MODE_DISABLED            = 0x00,
    MODE_IDLE                = 0x01,
    MODE_DAMPING             = 0x02,
    MODE_CALIBRATION         = 0x05,
    MODE_CURRENT             = 0x10,
    MODE_TORQUE              = 0x11,
    MODE_VELOCITY            = 0x12,
    MODE_POSITION            = 0x13,
    MODE_VABC_OVERRIDE       = 0x20,
    MODE_VALPHABETA_OVERRIDE = 0x21,
    MODE_VQD_OVERRIDE        = 0x22,
    MODE_DEBUG               = 0x80,
};

// ─── Error codes ──────────────────────────────────────────────────────────────
// Source: motor_controller_conf.h ErrorCode enum, confirmed against can_bus.py
enum ErrorCode : uint32_t {
    ERROR_NO_ERROR             = 0x0000u,
    ERROR_GENERAL              = 0x0001u,
    ERROR_ESTOP                = 0x0002u,
    ERROR_INITIALIZATION_ERROR = 0x0004u,
    ERROR_CALIBRATION_ERROR    = 0x0008u,
    ERROR_POWERSTAGE_ERROR     = 0x0010u,
    ERROR_INVALID_MODE         = 0x0020u,
    ERROR_WATCHDOG_TIMEOUT     = 0x0040u,
    ERROR_OVER_VOLTAGE         = 0x0080u,
    ERROR_OVER_CURRENT         = 0x0100u,
    ERROR_OVER_TEMPERATURE     = 0x0200u,
    ERROR_CAN_RX_FAULT         = 0x0400u,
    ERROR_CAN_TX_FAULT         = 0x0800u,
    ERROR_I2C_FAULT            = 0x1000u,
    ERROR_ENCODER_FAULT        = 0x2000u,  // Python adds this; not in firmware header
};

// ─── SDO parameter addresses ──────────────────────────────────────────────────
// Source: motor_controller_conf.h Parameter enum, confirmed against can_bus.py Parameter enum
// These are byte offsets into the MotorController firmware struct.
enum ParamId : uint16_t {
    PARAM_DEVICE_ID                                 = 0x000,
    PARAM_FIRMWARE_VERSION                          = 0x004,
    PARAM_WATCHDOG_TIMEOUT                          = 0x008,
    PARAM_FAST_FRAME_FREQUENCY                      = 0x00C,
    PARAM_MODE                                      = 0x010,
    PARAM_ERROR                                     = 0x014,
    PARAM_POSITION_CONTROLLER_UPDATE_COUNTER        = 0x018,
    PARAM_POSITION_CONTROLLER_GEAR_RATIO            = 0x01C,
    PARAM_POSITION_CONTROLLER_POSITION_KP           = 0x020,
    PARAM_POSITION_CONTROLLER_POSITION_KI           = 0x024,
    PARAM_POSITION_CONTROLLER_VELOCITY_KP           = 0x028,
    PARAM_POSITION_CONTROLLER_VELOCITY_KI           = 0x02C,
    PARAM_POSITION_CONTROLLER_TORQUE_LIMIT          = 0x030,
    PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT        = 0x034,
    PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER  = 0x038,
    PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER  = 0x03C,
    PARAM_POSITION_CONTROLLER_POSITION_OFFSET       = 0x040,
    PARAM_POSITION_CONTROLLER_TORQUE_TARGET         = 0x044,
    PARAM_POSITION_CONTROLLER_TORQUE_MEASURED       = 0x048,
    PARAM_POSITION_CONTROLLER_TORQUE_SETPOINT       = 0x04C,
    PARAM_POSITION_CONTROLLER_VELOCITY_TARGET       = 0x050,
    PARAM_POSITION_CONTROLLER_VELOCITY_MEASURED     = 0x054,
    PARAM_POSITION_CONTROLLER_VELOCITY_SETPOINT     = 0x058,
    PARAM_POSITION_CONTROLLER_POSITION_TARGET       = 0x05C,
    PARAM_POSITION_CONTROLLER_POSITION_MEASURED     = 0x060,
    PARAM_POSITION_CONTROLLER_POSITION_SETPOINT     = 0x064,
    PARAM_POSITION_CONTROLLER_POSITION_INTEGRATOR   = 0x068,
    PARAM_POSITION_CONTROLLER_VELOCITY_INTEGRATOR   = 0x06C,
    PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA   = 0x070,
    PARAM_CURRENT_CONTROLLER_I_LIMIT                = 0x074,
    PARAM_CURRENT_CONTROLLER_I_KP                   = 0x078,
    PARAM_CURRENT_CONTROLLER_I_KI                   = 0x07C,
    PARAM_CURRENT_CONTROLLER_I_A_MEASURED           = 0x080,
    PARAM_CURRENT_CONTROLLER_I_B_MEASURED           = 0x084,
    PARAM_CURRENT_CONTROLLER_I_C_MEASURED           = 0x088,
    PARAM_CURRENT_CONTROLLER_V_A_SETPOINT           = 0x08C,
    PARAM_CURRENT_CONTROLLER_V_B_SETPOINT           = 0x090,
    PARAM_CURRENT_CONTROLLER_V_C_SETPOINT           = 0x094,
    PARAM_CURRENT_CONTROLLER_I_ALPHA_MEASURED       = 0x098,
    PARAM_CURRENT_CONTROLLER_I_BETA_MEASURED        = 0x09C,
    PARAM_CURRENT_CONTROLLER_V_ALPHA_SETPOINT       = 0x0A0,
    PARAM_CURRENT_CONTROLLER_V_BETA_SETPOINT        = 0x0A4,
    PARAM_CURRENT_CONTROLLER_V_Q_TARGET             = 0x0A8,
    PARAM_CURRENT_CONTROLLER_V_D_TARGET             = 0x0AC,
    PARAM_CURRENT_CONTROLLER_V_Q_SETPOINT           = 0x0B0,
    PARAM_CURRENT_CONTROLLER_V_D_SETPOINT           = 0x0B4,
    PARAM_CURRENT_CONTROLLER_I_Q_TARGET             = 0x0B8,
    PARAM_CURRENT_CONTROLLER_I_D_TARGET             = 0x0BC,
    PARAM_CURRENT_CONTROLLER_I_Q_MEASURED           = 0x0C0,
    PARAM_CURRENT_CONTROLLER_I_D_MEASURED           = 0x0C4,
    PARAM_CURRENT_CONTROLLER_I_Q_SETPOINT           = 0x0C8,
    PARAM_CURRENT_CONTROLLER_I_D_SETPOINT           = 0x0CC,
    PARAM_CURRENT_CONTROLLER_I_Q_INTEGRATOR         = 0x0D0,
    PARAM_CURRENT_CONTROLLER_I_D_INTEGRATOR         = 0x0D4,
    PARAM_POWERSTAGE_HTIM                           = 0x0D8,
    PARAM_POWERSTAGE_HADC1                          = 0x0DC,
    PARAM_POWERSTAGE_HADC2                          = 0x0E0,
    PARAM_POWERSTAGE_ADC_READING_RAW                = 0x0E4,
    PARAM_POWERSTAGE_ADC_READING_OFFSET             = 0x0EC,
    PARAM_POWERSTAGE_UNDERVOLTAGE_THRESHOLD         = 0x0F4,
    PARAM_POWERSTAGE_OVERVOLTAGE_THRESHOLD          = 0x0F8,
    PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA       = 0x0FC,
    PARAM_POWERSTAGE_BUS_VOLTAGE_MEASURED           = 0x100,
    PARAM_MOTOR_POLE_PAIRS                          = 0x104,
    PARAM_MOTOR_TORQUE_CONSTANT                     = 0x108,
    PARAM_MOTOR_PHASE_ORDER                         = 0x10C,
    PARAM_MOTOR_MAX_CALIBRATION_CURRENT             = 0x110,
    PARAM_ENCODER_HI2C                              = 0x114,
    PARAM_ENCODER_I2C_BUFFER                        = 0x118,
    PARAM_ENCODER_I2C_UPDATE_COUNTER                = 0x11C,
    PARAM_ENCODER_CPR                               = 0x120,
    PARAM_ENCODER_POSITION_OFFSET                   = 0x124,
    PARAM_ENCODER_VELOCITY_FILTER_ALPHA             = 0x128,
    PARAM_ENCODER_POSITION_RAW                      = 0x12C,
    PARAM_ENCODER_N_ROTATIONS                       = 0x130,
    PARAM_ENCODER_POSITION                          = 0x134,
    PARAM_ENCODER_VELOCITY                          = 0x138,
    PARAM_ENCODER_FLUX_OFFSET                       = 0x13C,
    PARAM_ENCODER_FLUX_OFFSET_TABLE                 = 0x140,
};

// ─── SDO command bytes ────────────────────────────────────────────────────────
// Source: can_bus.py _SDO_CCS_WRITE / _SDO_CCS_READ
static constexpr uint8_t SDO_CMD_WRITE  = 0x20;  // CCS=1, expedited download request (host→node)
static constexpr uint8_t SDO_CMD_READ   = 0x40;  // CCS=2, upload initiate request   (host→node)
static constexpr uint8_t SDO_WRITE_ACK  = 0x60;  // download response byte 0, DLC=8   (node→host)
static constexpr uint8_t SDO_READ_RESP  = 0x43;  // upload response byte 0,   DLC=8   (node→host)

// ─── Ping magic byte ─────────────────────────────────────────────────────────
// Source: can_bus.py ping() — sends RECEIVE_PDO_1 with 0xCA, expects TRANSMIT_PDO_1 echo
static constexpr uint8_t PING_MAGIC = 0xCA;

// ─── Frame builder helpers ────────────────────────────────────────────────────
// These work with raw byte buffers to avoid UB from pointer aliasing.

// Write a float32 LE into buf[0..3]
inline void put_f32(uint8_t* buf, float v) {
    std::memcpy(buf, &v, 4);
}
// Read a float32 LE from buf[0..3]
inline float get_f32(const uint8_t* buf) {
    float v;
    std::memcpy(&v, buf, 4);
    return v;
}
// Write a uint32 LE into buf[0..3]
inline void put_u32(uint8_t* buf, uint32_t v) {
    std::memcpy(buf, &v, 4);
}
// Read a uint32 LE from buf[0..3]
inline uint32_t get_u32(const uint8_t* buf) {
    uint32_t v;
    std::memcpy(&v, buf, 4);
    return v;
}
// Read a uint16 LE from buf[0..1]
inline uint16_t get_u16(const uint8_t* buf) {
    uint16_t v;
    std::memcpy(&v, buf, 2);
    return v;
}

// ─── Frame structs ────────────────────────────────────────────────────────────
// All structs are packed and verified by static_assert.

// PDO2 TX (host → node, RECEIVE_PDO_2): position command
// Source: HANDOFF.md §2.1, can_bus.py send_pdo2()
struct __attribute__((packed)) Pdo2Tx {
    float position_target;   // rad, output-side (gear_ratio already applied)
    float velocity_ff;       // rad/s feedforward (ignored in POSITION mode)
};
static_assert(sizeof(Pdo2Tx) == 8, "Pdo2Tx must be 8 bytes");

// PDO2 RX (node → host, TRANSMIT_PDO_2): position feedback
// Source: HANDOFF.md §2.1, can_bus.py recv_pdo2()
struct __attribute__((packed)) Pdo2Rx {
    float position_measured; // rad, output-side
    float velocity_measured; // rad/s, output-side
};
static_assert(sizeof(Pdo2Rx) == 8, "Pdo2Rx must be 8 bytes");

// PDO4 RX (node → host, TRANSMIT_PDO_4): autonomous fast-frame
// Source: HANDOFF.md §2.1 — same layout as PDO2 RX
struct __attribute__((packed)) Pdo4Rx {
    float position;  // rad
    float velocity;  // rad/s
};
static_assert(sizeof(Pdo4Rx) == 8, "Pdo4Rx must be 8 bytes");

// PDO3 RX (node → host, TRANSMIT_PDO_3): calibration status (new firmware)
// Source: DAEMON_SPEC.md §2.5 — voltage_setpoint + phase_current_or_progress
struct __attribute__((packed)) Pdo3Rx {
    float voltage_or_progress;  // V during ramp; 0.0–1.0 progress during sweep
    float current_or_progress;  // A during ramp; 0.0–1.0 during sweep
};
static_assert(sizeof(Pdo3Rx) == 8, "Pdo3Rx must be 8 bytes");

// SDO request (host → node, RECEIVE_SDO): 8 bytes
// Source: can_bus.py _sdo_read / _sdo_write struct.pack("<BHB4x", cmd, param_id, 0)
// Layout: [cmd(1), param_id_lo(1), param_id_hi(1), pad(1), value(4)]
struct __attribute__((packed)) SdoRequest {
    uint8_t  cmd;       // SDO_CMD_READ or SDO_CMD_WRITE
    uint16_t param_id;  // LE uint16
    uint8_t  _pad;      // 0x00
    uint8_t  value[4];  // float32 or uint32 LE; zeros for reads
};
static_assert(sizeof(SdoRequest) == 8, "SdoRequest must be 8 bytes");

// SDO response (node → host, TRANSMIT_SDO): standard 8-byte CANopen upload response
// Byte 0: 0x43 (SDO_READ_RESP), bytes 1-2: param_id LE, byte 3: 0, bytes 4-7: value LE
struct __attribute__((packed)) SdoResponse {
    uint8_t  cmd;       // SDO_READ_RESP (0x43)
    uint16_t param_id;  // echoed from request
    uint8_t  _pad;      // 0x00
    uint8_t  value[4];  // float32 or uint32 LE
};
static_assert(sizeof(SdoResponse) == 8, "SdoResponse must be 8 bytes");

// NMT frame (host → node, FUNC_NMT): mode change
// Source: can_bus.py set_mode() — struct.pack("<BB", mode, device_id)
struct __attribute__((packed)) NmtFrame {
    uint8_t mode;       // MotorMode value
    uint8_t device_id;  // addressed node; 0 = broadcast
};
static_assert(sizeof(NmtFrame) == 2, "NmtFrame must be 2 bytes");

// Heartbeat RX (node → host, FUNC_HEARTBEAT, new firmware): mode-change ACK
// Source: DAEMON_SPEC.md §2.5 — 5 bytes: mode(1) + error_code(4)
struct __attribute__((packed)) HeartbeatRx {
    uint8_t  mode;        // MotorMode
    uint32_t error_code;  // ErrorCode bitmask LE
};
static_assert(sizeof(HeartbeatRx) == 5, "HeartbeatRx must be 5 bytes");

// EMCY frame (node → host, FUNC_SYNC_EMCY, new firmware): fault broadcast
// Source: DAEMON_SPEC.md §2.5 — 4 bytes: error_code uint32 LE
struct __attribute__((packed)) EmcyFrame {
    uint32_t error_code;
};
static_assert(sizeof(EmcyFrame) == 4, "EmcyFrame must be 4 bytes");

// Flash command (host → node, FUNC_FLASH)
// Source: HANDOFF.md §2.1, can_bus.py store_to_flash / load_from_flash
struct __attribute__((packed)) FlashCmd {
    uint8_t op;  // 1 = store, 2 = load
};
static_assert(sizeof(FlashCmd) == 1, "FlashCmd must be 1 byte");
