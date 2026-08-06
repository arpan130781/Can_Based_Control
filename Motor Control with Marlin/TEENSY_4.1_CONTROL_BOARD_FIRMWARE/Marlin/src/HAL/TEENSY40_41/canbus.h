#pragma once

#include <stdint.h>

void canbus_init();
void can_send_packet(uint16_t id, uint8_t len, uint8_t *data);
