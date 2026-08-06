// ============================================================
// can_protocol.h
// Shared CAN IDs and command bytes — Teensy master & STM32 slave
// ============================================================
#pragma once
#include <stdint.h>

// ── CAN Bus ──────────────────────────────────────────────────
#define CAN_BAUDRATE       500000UL   // 500 kbps, must match slave

// ── CAN IDs ──────────────────────────────────────────────────
// Master TX  :  MOTOR_CMD_ID_BASE + slaveId  (e.g. 0x101 for slave 1)
// Master RX  :  MOTOR_ACK_ID_BASE + slaveId  (e.g. 0x201 for slave 1)
#define MOTOR_CMD_ID_BASE  0x100
#define MOTOR_ACK_ID_BASE  0x200

// ── Command Bytes ─────────────────────────────────────────────
#define CMD_START_MOTOR    0x10
#define CMD_STOP_MOTOR     0x11
#define CMD_SET_SPEED      0x12
#define CMD_SET_DIRECTION  0x13
#define CMD_ESTOP          0x14
#define CMD_PING           0x15
#define CMD_ACK            0x16