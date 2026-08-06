/**
 * custom/M756.cpp  — Generic CAN axis motion bridge (Teensy / Marlin side)
 *
 * Forwards any axis move or home event to the correct slave over CAN.
 * The slave is identified by CAN_AXIS_SLAVE_ID (default 1).
 *
 * HOW IT WORKS
 * ────────────
 * Marlin's get_destination_from_command() fills destination[axis] with
 * the resolved ABSOLUTE position for every axis in the G0/G1 command,
 * regardless of whether G90 or G91 mode is active.
 *
 * G0_G1.cpp loops over every configured axis after the planner call.
 * For each axis word that was present in the command it calls:
 *
 *     can_axis_move(axis_index, destination[axis], was_relative)
 *
 * G28.cpp calls:
 *
 *     can_axis_home(axis_index)
 *
 * for each axis that was actually homed.
 *
 * ADDING A NEW AXIS
 * ─────────────────
 * Nothing changes here.  Add an entry to axis_config[] on the slave and
 * reflash the slave.  The Teensy code is already complete for any axis
 * Marlin knows about.
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(CAN_AXIS_SUPPORT)

#include "../gcode.h"
#include "../../core/serial.h"
#include "../../module/motion.h"
#include "can_motion_protocol.h"

// Provided by HAL/TEENSY40_41/canbus.cpp
extern void can_send_packet(uint16_t id, uint8_t len, uint8_t *data);

// ── Axis name helper for serial debug output ──────────────────────────────
static char axis_name(uint8_t axis) {
  const char names[] = { 'X', 'Y', 'Z', 'A', 'B', 'C', 'U', 'V', 'W' };
  return (axis < sizeof(names)) ? names[axis] : '?';
}

// ─────────────────────────────────────────────────────────────────────────
// can_axis_move
//
// axis      : Marlin AxisEnum index (X=0 Y=1 Z=2 A/I=3 B/J=4 …)
// target_mm : absolute target position in mm or degrees
// relative  : true when the originating gcode used G91 (informational)
// ─────────────────────────────────────────────────────────────────────────
void can_axis_move(const uint8_t axis, const float target_mm, const bool relative) {

  const uint16_t can_id = (uint16_t)(CAN_CMD_ID_BASE + CAN_AXIS_SLAVE_ID);

  union { float f; uint8_t b[4]; } u;
  u.f = target_mm;

  uint8_t payload[7] = {
    CMD_AXIS_MOVE,
    axis,
    (uint8_t)(relative ? 1u : 0u),
    u.b[0], u.b[1], u.b[2], u.b[3]
  };

  can_send_packet(can_id, 7, payload);

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM("CAN AXIS_MOVE -> 0x");
  SERIAL_PRINT(can_id, PrintBase::Hex);
  SERIAL_ECHOPGM("  axis="); SERIAL_CHAR(axis_name(axis));
  SERIAL_ECHOPGM("  mode="); SERIAL_ECHO(relative ? "REL" : "ABS");
  SERIAL_ECHOPGM("  target="); SERIAL_ECHOLN(target_mm);
}

// ─────────────────────────────────────────────────────────────────────────
// can_axis_home
//
// Tells the slave "axis X/Y/Z/A/B just finished homing — reset your
// position counter to 0".  The slave must NOT move its motor.
// ─────────────────────────────────────────────────────────────────────────
void can_axis_home(const uint8_t axis) {

  const uint16_t can_id = (uint16_t)(CAN_CMD_ID_BASE + CAN_AXIS_SLAVE_ID);

  uint8_t payload[2] = { CMD_AXIS_HOME, axis };
  can_send_packet(can_id, 2, payload);

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM("CAN AXIS_HOME -> 0x");
  SERIAL_PRINT(can_id, PrintBase::Hex);
  SERIAL_ECHOPGM("  axis="); SERIAL_CHAR(axis_name(axis)); SERIAL_EOL();
}

#endif // CAN_AXIS_SUPPORT
