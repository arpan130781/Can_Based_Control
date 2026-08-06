#include "../../inc/MarlinConfig.h"

#if ENABLED(CAN_FEEDER_SUPPORT)

#include "../gcode.h"
#include "../../core/serial.h"

// ---- CAN send function (Teensy CAN layer se link hoga) ----
extern void can_send_packet(uint16_t id, uint8_t len, uint8_t *data);

void GcodeSuite::M755() {

  SERIAL_ECHOLNPGM(">>> M755 CALLED <<<");

  // ----- Parameter check -----
  if (!parser.seen('P') || !parser.seen('S')) {
    SERIAL_ECHOLNPGM("Error: M755 requires P<index> S<0-255>");
    return;
  }

  const uint8_t feeder = parser.byteval('P');  // ✅ correct
  const uint8_t value  = parser.byteval('S');  // ✅ correct

  if (feeder >= CAN_FEEDER_MAX) {
    SERIAL_ECHOLNPGM("Error: Invalid feeder index");
    return;
  }

  const uint16_t can_id = CAN_FEEDER_BASE_ID + feeder;
  uint8_t data[1] = { value };

  can_send_packet(can_id, 1, data);

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM("CAN Feeder ");
  SERIAL_ECHO(feeder);
  SERIAL_ECHOPGM(" -> ");
  SERIAL_ECHOLN(value);
}

#endif // CAN_FEEDER_SUPPORT
