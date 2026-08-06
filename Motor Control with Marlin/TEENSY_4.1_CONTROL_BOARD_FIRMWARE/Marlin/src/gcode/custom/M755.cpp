/**
 * Marlin Firmware – Custom Extension
 *
 * M755 L<slave_id> D<feeder_index> S<speed 0-255>
 *
 * Lightweight alternate to M155 for setting a feeder's speed directly
 * as an 8-bit PWM-style value (0-255) instead of an RPM value.
 *
 * BUGFIX (see project history):
 *   The previous implementation sent a single raw byte to CAN ID
 *   (CAN_FEEDER_BASE_ID + feeder) = 0x200 + feeder. That ID range
 *   collides with ACK_ID_BASE (0x200 + slave_id) used by the slave's
 *   ACK replies, AND the slave only ever processes frames whose ID
 *   equals (CMD_ID_BASE + SLAVE_ID) -- see STM32F103_Slave/src/main.cpp
 *   loop(): `if (rxId == CMD_ID_BASE + SLAVE_ID)`. Frames sent to any
 *   other ID, including the old 0x200+feeder scheme, are silently
 *   logged as "ignored" by the slave and never reach processMessage().
 *   On top of that, the old payload was a single byte with no command
 *   byte and no feeder index, which the slave's processMessage()
 *   would have misinterpreted as the *command* byte even if it had
 *   been received. M755 was therefore completely non-functional.
 *
 * FIX: M755 now reuses the single shared protocol (can_motion_protocol.h)
 * and addresses the slave the same way M155 does: CMD_ID_BASE + slave_id.
 * Payload matches the slave's CMD_START_MOTOR parser exactly:
 *   [0] CMD_START_MOTOR  [1] feeder  [2] speed_lo  [3] speed_hi(=0)
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(CAN_FEEDER_SUPPORT)

#include "../gcode.h"
#include "../../core/serial.h"
#include "can_motion_protocol.h"   // single source of truth — do not redefine

// Provided by HAL/TEENSY40_41/canbus.cpp
extern void can_send_packet(uint16_t id, uint8_t len, uint8_t *data);

void GcodeSuite::M755() {

  if (!parser.seen('L') || !parser.seen('D') || !parser.seen('S')) {
    SERIAL_ECHOLNPGM("M755 Error: L, D, and S are all required.");
    SERIAL_ECHOLNPGM("  Usage: M755 L<slave_id> D<feeder 0-19> S<speed 0-255>");
    return;
  }

  const uint8_t slave_id = parser.byteval('L');
  const uint8_t feeder   = parser.byteval('D');
  const uint8_t speed    = parser.byteval('S');

  if (slave_id == 0) {
    SERIAL_ECHOLNPGM("M755 Error: L (slave ID) must be >= 1.");
    return;
  }

  // NOTE: must match NUMBER_OF_FEEDERS in STM32F103_Slave/src/main.cpp
  if (feeder >= CAN_FEEDER_MAX) {
    SERIAL_ECHOLNPGM("M755 Error: D (feeder index) out of range.");
    return;
  }

  const uint16_t can_id = (uint16_t)(CAN_CMD_ID_BASE + slave_id);

  uint8_t payload[4] = {
    CMD_START_MOTOR,    // data[0]: command
    feeder,             // data[1]: feeder index
    speed,              // data[2]: speed low byte
    0x00                // data[3]: speed high byte (unused for 8-bit speed)
  };

  can_send_packet(can_id, 4, payload);

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM("M755 -> CAN 0x");
  SERIAL_PRINT(can_id, PrintBase::Hex);
  SERIAL_ECHOPGM("  Slave="); SERIAL_ECHO(slave_id);
  SERIAL_ECHOPGM("  Feeder="); SERIAL_ECHO(feeder);
  SERIAL_ECHOPGM("  Speed="); SERIAL_ECHOLN(speed);
}

#endif // CAN_FEEDER_SUPPORT
