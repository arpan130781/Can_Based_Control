/**
 * Marlin 3D Printer Firmware – Custom Extension
 *
 * Custom M155 handler: M155 L<slave_id> D<feeder_index> S<rpm>
 *
 * ── Parameter semantics ──────────────────────────────────────────────────
 *
 *   L  : Slave ID      — must match SLAVE_ID on the target Blue Pill board.
 *                         CAN frame is sent to (CMD_ID_BASE + L) = 0x100 + L.
 *   D  : Feeder index  — 0–26 (27 feeders per slave, see feederPins[] array).
 *   S  : Target RPM    — 0 = stop, 1–65000 = desired RPM.
 *                         The slave looks this up in its calibration table and
 *                         calls analogWrite() with the corresponding PWM duty.
 *
 * ── CAN frame format (4 bytes, as parsed by slave processMessage()) ──────
 *
 *   data[0]  CMD byte       : CMD_START_MOTOR (0x10)
 *   data[1]  Feeder index   : D parameter value (0–26)
 *   data[2]  RPM low byte   : S & 0xFF
 *   data[3]  RPM high byte  : (S >> 8) & 0xFF
 *
 *   The slave reassembles: rpm = (uint16_t)data[2] | ((uint16_t)data[3] << 8)
 *   then maps rpm → PWM via its rpmToPwm() calibration table.
 *
 * ── Slave CAN ID scheme ──────────────────────────────────────────────────
 *
 *   CMD_ID_BASE = 0x100
 *   The slave listens on: CMD_ID_BASE + SLAVE_ID  → e.g. slave 1 = 0x101
 *   The slave ACKs on:    ACK_ID_BASE + SLAVE_ID  → e.g. slave 1 = 0x201
 *
 * ── Example ──────────────────────────────────────────────────────────────
 *
 *   M155 L1 D0 S300
 *
 *   → CAN ID  = 0x101  (slave 1)
 *   → payload = [0x10, 0x00, 0x2C, 0x01]
 *              = CMD_START_MOTOR, feeder 0, rpm_low=0x2C, rpm_high=0x01
 *   → slave reassembles 300 RPM, looks up calibration table → PWM duty
 *
 * ── Guard ────────────────────────────────────────────────────────────────
 *
 *   This file compiles only when CUSTOM_M155_COMMAND is defined.
 *   The stock AUTO_REPORT_TEMPERATURES M155 (temp/M155.cpp) is guarded by
 *   ALL(AUTO_REPORT_TEMPERATURES, HAS_TEMP_SENSOR) && DISABLED(CUSTOM_M155_COMMAND)
 *   so there is never a duplicate symbol.
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(CUSTOM_M155_COMMAND)

#include "../gcode.h"
#include "../../core/serial.h"
#include "can_motion_protocol.h"   // BUGFIX: single source of truth — was
                                    // previously re-defined locally below,
                                    // risking silent protocol drift between
                                    // the Teensy and STM32 sides.

#define SLAVE_CMD_ID_BASE  CAN_CMD_ID_BASE
#define SLAVE_ACK_ID_BASE  CAN_ACK_ID_BASE

// BUGFIX: was hardcoded to 27, but the slave (STM32F103_Slave/src/main.cpp)
// only has 20 feeder pins (NUMBER_OF_FEEDERS = 20, indices 0-19). Values
// 20-26 were silently accepted here and then silently dropped by the
// slave's own bounds check, so M155 reported success for feeders that
// never moved. Now driven by the single Configuration_adv.h setting.
#define FEEDER_COUNT_PER_SLAVE  CUSTOM_M155_FEEDER_MAX
#define MAX_RPM                 65000u  // upper sanity limit

// ── CAN send function provided by the Teensy CAN layer ───────────────────
extern void can_send_packet(uint16_t id, uint8_t len, uint8_t *data);

// ── Forward declaration ───────────────────────────────────────────────────
static void handle_M155_feeder(uint8_t slave_id,
                                uint8_t feeder_index,
                                uint16_t rpm);

// ---------------------------------------------------------------------------
// GcodeSuite::M155
//
// Parses:   M155 L<slave_id> D<feeder_index> S<rpm>
//
// S is now uint16 (0–65000 RPM). The slave's calibration table converts
// RPM to a PWM duty cycle via rpmToPwm().
// ---------------------------------------------------------------------------
void GcodeSuite::M155() {

  // ── 1. Presence check ────────────────────────────────────────────────────
  if (!parser.seen('L') || !parser.seen('D') || !parser.seen('S')) {
    SERIAL_ECHOLNPGM("M155 Error: L, D, and S are all required.");
    SERIAL_ECHOLNPGM("  Usage: M155 L<slave_id> D<feeder 0-26> S<rpm 0-65000>");
    SERIAL_ECHOLNPGM("  Example: M155 L1 D0 S300  (slave 1, feeder 0, 300 RPM)");
    return;
  }

  // ── 2. Value extraction ──────────────────────────────────────────────────
  const uint8_t  slave_id     = parser.byteval('L');
  const uint8_t  feeder_index = parser.byteval('D');
  const uint16_t rpm          = parser.ushortval('S');  // now uint16, accepts 0–65535

  // ── 3. Validation ────────────────────────────────────────────────────────
  if (slave_id == 0) {
    SERIAL_ECHOLNPGM("M155 Error: L (slave ID) must be >= 1.");
    return;
  }

  if (feeder_index >= FEEDER_COUNT_PER_SLAVE) {
    SERIAL_ECHOLNPGM("M155 Error: D (feeder index) out of range.");
    return;
  }

  if (rpm > MAX_RPM) {
    SERIAL_ECHOLNPGM("M155 Error: S (RPM) must be 0-65000.");
    return;
  }

  // ── 4. Dispatch ──────────────────────────────────────────────────────────
  handle_M155_feeder(slave_id, feeder_index, rpm);
}

// ---------------------------------------------------------------------------
// handle_M155_feeder
//
// Packs the 16-bit RPM value into the two payload bytes the slave already
// parses as a uint16_t:
//   rpm = (uint16_t)data[2] | ((uint16_t)data[3] << 8)
//
// The slave's rpmToPwm() then maps that RPM to a PWM duty (0–255).
// ---------------------------------------------------------------------------
static void handle_M155_feeder(const uint8_t  slave_id,
                                const uint8_t  feeder_index,
                                const uint16_t rpm) {

  const uint16_t can_id = (uint16_t)(SLAVE_CMD_ID_BASE + slave_id);

  uint8_t payload[4] = {
    CMD_START_MOTOR,                    // data[0]: command
    feeder_index,                       // data[1]: feeder index
    (uint8_t)(rpm & 0xFF),              // data[2]: RPM low byte
    (uint8_t)((rpm >> 8) & 0xFF)        // data[3]: RPM high byte
  };

  can_send_packet(can_id, 4, payload);

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM("M155 -> CAN 0x");
  SERIAL_PRINT(can_id, PrintBase::Hex);
  SERIAL_ECHOPGM("  CMD=0x10 (START_MOTOR)");
  SERIAL_ECHOPGM("  Slave=");   SERIAL_ECHO(slave_id);
  SERIAL_ECHOPGM("  Feeder=");  SERIAL_ECHO(feeder_index);
  SERIAL_ECHOPGM("  RPM=");     SERIAL_ECHOLN(rpm);
  SERIAL_ECHOPGM("  Payload=[0x10,");
  SERIAL_PRINT(feeder_index, PrintBase::Hex);
  SERIAL_ECHOPGM(",");
  SERIAL_PRINT((uint8_t)(rpm & 0xFF), PrintBase::Hex);
  SERIAL_ECHOPGM(",");
  SERIAL_PRINT((uint8_t)((rpm >> 8) & 0xFF), PrintBase::Hex);
  SERIAL_ECHOLNPGM("]");
}

#endif // CUSTOM_M155_COMMAND