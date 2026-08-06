#pragma once
/**
 * can_motion_protocol.h
 *
 * Single source of truth for the CAN motion protocol spoken between the
 * Teensy 4.1 master (Marlin) and any STM32F103 slave.
 *
 * Include this file on BOTH sides:
 *   - Teensy  : Marlin/src/gcode/custom/can_motion_protocol.h
 *   - STM32   : STM32F103 Slave Code/src/can_motion_protocol.h
 *
 * ═══════════════════════════════════════════════════════════════════════
 * DESIGN PHILOSOPHY — "self-describing frames"
 * ═══════════════════════════════════════════════════════════════════════
 *
 * Every motion frame carries:
 *   [0]  command byte  — what to do
 *   [1]  axis index    — which axis (0=X 1=Y 2=Z 3=A 4=B …)
 *   [2]  flags         — bit0: 1=was-relative, 0=absolute
 *   [3..6] float LE    — absolute target position in mm (or degrees)
 *
 * Because the axis index is in the frame, adding a new axis requires
 * ZERO changes on the Teensy side — the master always loops over all
 * configured axes and sends the same frame structure.
 * On the slave, you add one entry to the axis_config[] table and
 * recompile — nothing else.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * CAN ID allocation
 * ═══════════════════════════════════════════════════════════════════════
 *
 *   0x100 + slave_id   →  command frames  (Teensy → Slave)
 *   0x200 + slave_id   →  ACK frames      (Slave  → Teensy)
 *
 * ═══════════════════════════════════════════════════════════════════════
 * Command byte table
 * ═══════════════════════════════════════════════════════════════════════
 */

// ── Feeder commands (unchanged) ───────────────────────────────────────────
#define CMD_START_MOTOR    0x10u   // data[1]=feeder, data[2..3]=rpm LE
#define CMD_STOP_MOTOR     0x11u   // data[1]=feeder
#define CMD_SET_SPEED      0x12u   // data[1]=feeder, data[2..3]=rpm LE
#define CMD_SET_DIRECTION  0x13u   // data[1]=feeder, data[2]=dir
#define CMD_ESTOP          0x14u   // no payload — stop everything
#define CMD_PING           0x15u   // no payload — slave replies with ACK

// ── Generic axis motion commands ──────────────────────────────────────────
/**
 * CMD_AXIS_MOVE  0x20
 *
 * Sent by Teensy whenever a G0/G1 command contains a word for an axis
 * that is mapped to this slave.
 *
 * Frame layout (7 bytes):
 *   [0]     0x20            CMD_AXIS_MOVE
 *   [1]     axis_index      0=X 1=Y 2=Z 3=A(I) 4=B(J) 5=C(K) …
 *   [2]     flags           bit0: 1=move originated from G91, 0=G90
 *                           (informational only — target is always absolute)
 *   [3]     float byte 0    \
 *   [4]     float byte 1     > IEEE-754 single, little-endian
 *   [5]     float byte 2     > absolute target in mm (or degrees)
 *   [6]     float byte 3    /
 */
#define CMD_AXIS_MOVE      0x20u

/**
 * CMD_AXIS_HOME  0x21
 *
 * Sent by Teensy at the end of G28 for each axis that was homed.
 * Tells the slave to reset its position counter to 0 without moving.
 *
 * Frame layout (2 bytes):
 *   [0]     0x21            CMD_AXIS_HOME
 *   [1]     axis_index      same encoding as CMD_AXIS_MOVE
 */
#define CMD_AXIS_HOME      0x21u

// ── Axis index encoding ───────────────────────────────────────────────────
// These match Marlin's AxisEnum order (X=0, Y=1, Z=2, I=3, J=4, K=5 …)
// Marlin labels I/J/K as A/B/C when AXIS4_NAME='A', AXIS5_NAME='B' etc.
#define CAN_AXIS_X   0u
#define CAN_AXIS_Y   1u
#define CAN_AXIS_Z   2u
#define CAN_AXIS_A   3u   // Marlin I_AXIS, named 'A' via AXIS4_NAME
#define CAN_AXIS_B   4u   // Marlin J_AXIS, named 'B' via AXIS5_NAME
#define CAN_AXIS_C   5u   // Marlin K_AXIS, named 'C' via AXIS6_NAME

// ── CAN bus IDs ───────────────────────────────────────────────────────────
#define CAN_CMD_ID_BASE   0x100u
#define CAN_ACK_ID_BASE   0x200u
