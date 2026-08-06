#include "canbus.h"
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// NOTE: onTransmit() is intentionally NOT used here. The vendored
// FlexCAN_T4 version pulled in by this project's platformio.ini does
// not expose onTransmit() (it's only present in newer releases of the
// library — see the grblHAL project's vendored copy for comparison).
// onTransmit() is a mitigation for TX-mailbox starvation over many
// sustained sends; it is not required to get a single frame onto the
// bus, so it's safe to omit while we're only sending occasional M155
// motor commands. If you upgrade FlexCAN_T4 later, onTransmit() can be
// added back the same way it's used in the grblHAL feeder_m155.cpp plugin.

static void can_rx_handler(const CAN_message_t &msg) {
  (void)msg; // master doesn't need to act on ACKs today; placeholder for future use
}

void canbus_init() {
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(can_rx_handler);
  Can0.mailboxStatus();
}

void can_send_packet(uint16_t id, uint8_t len, uint8_t *data) {
  // BUGFIX: CAN_message_t.buf is only 8 bytes. Without this clamp, a
  // caller that passes len > 8 (e.g. a future protocol change) would
  // memcpy past the end of msg.buf and corrupt adjacent stack memory.
  if (len > 8) len = 8;
  CAN_message_t msg;
  msg.id = id;
  msg.flags.extended = 0;
  msg.len = len;
  memcpy(msg.buf, data, len);
  Can0.write(msg);
}