/**
 * Marlin 3D Printer Firmware – Custom Extension
 *
 * Custom M155 handler: M155 L<slave_id> D<feeder_index> S<speed_0-255>
 *
 * ── Parameter semantics (derived from STM32F103 slave main.cpp) ──────────
 *
 *   L  : Slave ID      — must match SLAVE_ID on the target Blue Pill board.
 *                         CAN frame is sent to (CMD_ID_BASE + L) = 0x100 + L.
 *   D  : Feeder index  — 0–26 (27 feeders per slave, see feederPins[] array).
 *   S  : Speed / PWM   — 0 = off, 1–254 = analogWrite PWM duty, 255 = full ON.
 *                         The slave stores this in currentValues[feeder] and
 *                         calls analogWrite() or digitalWrite() accordingly.
 *
 * ── CAN frame format (4 bytes, as parsed by slave processMessage()) ──────
 *
 *   data[0]  CMD byte        : CMD_START_MOTOR (0x10) — starts the feeder
 *   data[1]  Feeder index    : D parameter value (0–26)
 *   data[2]  Speed low byte  : S parameter value (uint8_t, 0–255)
 *   data[3]  Speed high byte : 0x00  (rpm is uint16_t on slave; we send 8-bit)
 *
 * ── Slave CAN ID scheme (from slave main.cpp) ────────────────────────────
 *
 *   CMD_ID_BASE = 0x100
 *   The slave listens on: CMD_ID_BASE + SLAVE_ID  → e.g. slave 1 = 0x101
 *   The slave ACKs on:    ACK_ID_BASE + SLAVE_ID  → e.g. slave 1 = 0x201
 *
 * ── Example ──────────────────────────────────────────────────────────────
 *
 *   M155 L1 D0 S255
 *
 *   → CAN ID  = 0x101  (slave 1)
 *   → payload = [0x10, 0x00, 0xFF, 0x00]
 *              = CMD_START_MOTOR, feeder 0, speed 255, high-byte 0
 *
 *   Slave receives, enters setFeeder(0, 255), calls digitalWrite(PB12, HIGH)
 *   (feederPins[0] = PB12), sets currentValues[0] = 255, sends ACK 0x201.
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

// ── Slave protocol constants (mirror of slave main.cpp defines) ───────────
// Keep these in sync with the STM32F103 slave firmware.
#define CMD_START_MOTOR   0x10u
#define CMD_STOP_MOTOR    0x11u
#define CMD_SET_SPEED     0x12u
#define CMD_SET_DIRECTION 0x13u
#define CMD_ESTOP         0x14u

#define SLAVE_CMD_ID_BASE  0x100u   // slave listens on 0x100 + SLAVE_ID
#define SLAVE_ACK_ID_BASE  0x200u   // slave ACKs on    0x200 + SLAVE_ID (info only)

#define FEEDER_COUNT_PER_SLAVE  27u // feederPins[] has 27 entries on each slave

// ── CAN send function provided by the Teensy CAN layer ───────────────────
// Declared extern so this file does not need to include the Teensy-specific
// CAN header directly, matching the same pattern used by M755.cpp.
extern void can_send_packet(uint16_t id, uint8_t len, uint8_t *data);

// ── Forward declaration of the dedicated handler ─────────────────────────
static void handle_M155_feeder(uint8_t slave_id,
                                uint8_t feeder_index,
                                uint8_t speed);

// ---------------------------------------------------------------------------
// GcodeSuite::M155
//
// Parses:   M155 L<slave_id> D<feeder_index> S<speed 0-255>
//
// All three parameters are mandatory.  Missing or out-of-range values
// print a descriptive error and return without touching the CAN bus.
// ---------------------------------------------------------------------------
void GcodeSuite::M155() {

  // ── 1. Presence check ────────────────────────────────────────────────────
  if (!parser.seen('L') || !parser.seen('D') || !parser.seen('S')) {
    SERIAL_ECHOLNPGM("M155 Error: L, D, and S are all required.");
    SERIAL_ECHOLNPGM("  Usage: M155 L<slave_id> D<feeder 0-26> S<speed 0-255>");
    SERIAL_ECHOLNPGM("  Example: M155 L1 D0 S255  (slave 1, feeder 0, full speed)");
    return;
  }

  // ── 2. Value extraction ──────────────────────────────────────────────────
  // Presence of L, D, and S is already guaranteed by the parser.seen()
  // check above, so no "missing value" sentinel is needed here. (A
  // previous version used 0xFF as that sentinel — but 0xFF == 255 in
  // decimal, which collided with legitimate full-speed values like S255
  // and incorrectly rejected them as "missing". Removed.)
  const uint8_t slave_id      = parser.byteval('L');
  const uint8_t feeder_index  = parser.byteval('D');
  const uint8_t speed         = parser.byteval('S');

  // ── 3. Validation ────────────────────────────────────────────────────────

  // L: slave ID — must be a non-zero value (slave IDs start at 1 in the
  // slave firmware: SLAVE_ID = 1).  Upper bound is open; add a cap if you
  // have a fixed bus size (e.g. slave_id > 8).
  if (slave_id == 0) {
    SERIAL_ECHOLNPGM("M155 Error: L (slave ID) must be >= 1 (matches SLAVE_ID in slave firmware).");
    return;
  }

  // D: feeder index 0–26 (27 feeders per slave, feederPins[] has 27 entries)
  if (feeder_index >= FEEDER_COUNT_PER_SLAVE) {
    SERIAL_ECHOLNPGM("M155 Error: D (feeder index) must be 0–26.");
    return;
  }

  // S: speed 0–255. uint8_t already constrains the range; nothing further
  // to validate here.
  // S=0     → slave calls stopFeeder() / digitalWrite LOW
  // S=1–254 → slave calls analogWrite() (PWM duty cycle)
  // S=255   → slave calls digitalWrite HIGH (full on)

  // ── 4. Dispatch to CAN handler ───────────────────────────────────────────
  handle_M155_feeder(slave_id, feeder_index, speed);
}

// ---------------------------------------------------------------------------
// handle_M155_feeder
//
// Builds and transmits the 4-byte CAN frame that the STM32F103 slave
// expects in its processMessage() function.
//
// Frame layout (matches slave processMessage() parsing):
//   data[0] = CMD_START_MOTOR (0x10)
//   data[1] = feeder_index
//   data[2] = speed (low byte of uint16_t rpm)
//   data[3] = 0x00  (high byte of uint16_t rpm — always 0 for 8-bit speed)
//
// CAN ID = CMD_ID_BASE + slave_id  (e.g. slave 1 → 0x101)
// ---------------------------------------------------------------------------
static void handle_M155_feeder(const uint8_t slave_id,
                                const uint8_t feeder_index,
                                const uint8_t speed) {

  const uint16_t can_id = (uint16_t)(SLAVE_CMD_ID_BASE + slave_id);

  // 4-byte payload — slave unpacks as:
  //   cmd    = data[0]
  //   feeder = data[1]
  //   rpm    = (uint16_t)data[2] | ((uint16_t)data[3] << 8)
  uint8_t payload[4] = {
    CMD_START_MOTOR,   // data[0]: command
    feeder_index,      // data[1]: which feeder on this slave
    speed,             // data[2]: speed low byte  (0–255)
    0x00               // data[3]: speed high byte (always 0)
  };

  // Transmit via the Teensy CAN layer (same function used by M755)
  can_send_packet(can_id, 4, payload);

  // Confirm on host serial
  SERIAL_ECHO_START();
  SERIAL_ECHOPGM("M155 -> CAN 0x");
  SERIAL_PRINT(can_id, PrintBase::Hex);
  SERIAL_ECHOPGM("  CMD=0x10 (START_MOTOR)");
  SERIAL_ECHOPGM("  Slave=");   SERIAL_ECHO(slave_id);
  SERIAL_ECHOPGM("  Feeder=");  SERIAL_ECHO(feeder_index);
  SERIAL_ECHOPGM("  Speed=");   SERIAL_ECHOLN(speed);
  SERIAL_ECHOLNPGM("  Slave will ACK on CAN ID 0x2", slave_id);
}

#endif // CUSTOM_M155_COMMAND