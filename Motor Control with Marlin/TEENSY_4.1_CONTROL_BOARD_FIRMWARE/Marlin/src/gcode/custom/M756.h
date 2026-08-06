#pragma once
/**
 * custom/M756.h  — Generic CAN axis motion bridge (Teensy / Marlin side)
 *
 * Two functions are called from the GCode motion handlers:
 *
 *   can_axis_move(axis, target_mm, relative)
 *     Called from G0_G1.cpp for every axis word seen in a G0/G1 command
 *     that has a slave mapping.  Sends CMD_AXIS_MOVE (0x20).
 *
 *   can_axis_home(axis)
 *     Called from G28.cpp for each axis that was homed.
 *     Sends CMD_AXIS_HOME (0x21).
 *
 * Adding a new axis (e.g. C) requires ZERO changes here or in M756.cpp.
 * Only the slave's axis_config[] table needs updating.
 *
 * Guard: compiled only when CAN_AXIS_SUPPORT is defined in Configuration_adv.h
 */

#include "../gcode.h"
#include "can_motion_protocol.h"

#if ENABLED(CAN_AXIS_SUPPORT)
  void can_axis_move(uint8_t axis, float target_mm, bool relative);
  void can_axis_home(uint8_t axis);
#endif
