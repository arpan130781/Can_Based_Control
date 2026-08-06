/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../MarlinCore.h"

#if ALL(FWRETRACT, FWRETRACT_AUTORETRACT)
  #include "../../feature/fwretract.h"
#endif

#include "../../sd/cardreader.h"

#if ENABLED(NANODLP_Z_SYNC)
  #include "../../module/planner.h"
#endif

// ── Generic CAN axis bridge ───────────────────────────────────────────────
#if ENABLED(CAN_AXIS_SUPPORT)
  #include "../custom/M756.h"
#endif
// ─────────────────────────────────────────────────────────────────────────

extern xyze_pos_t destination;

#if ENABLED(VARIABLE_G0_FEEDRATE)
  feedRate_t fast_move_feedrate = MMM_TO_MMS(G0_FEEDRATE);
#endif

/**
 * G0, G1: Coordinated movement of X Y Z E axes (and extra axes A B C …)
 */
void GcodeSuite::G0_G1(TERN_(HAS_FAST_MOVES, const bool fast_move/*=false*/)) {
  if (!MOTION_CONDITIONS) return;

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_RUNNING));

  #ifdef G0_FEEDRATE
    feedRate_t old_feedrate;
    #if ENABLED(VARIABLE_G0_FEEDRATE)
      if (fast_move) {
        old_feedrate = feedrate_mm_s;
        feedrate_mm_s = fast_move_feedrate;
      }
    #endif
  #endif

  // After this call, destination[i] holds the resolved ABSOLUTE target
  // for every axis (G90/G91 is already applied).
  get_destination_from_command();

  #ifdef G0_FEEDRATE
    if (fast_move) {
      #if ENABLED(VARIABLE_G0_FEEDRATE)
        fast_move_feedrate = feedrate_mm_s;
      #else
        old_feedrate = feedrate_mm_s;
        feedrate_mm_s = MMM_TO_MMS(G0_FEEDRATE);
      #endif
    }
  #endif

  #if BOTH(FWRETRACT, FWRETRACT_AUTORETRACT)
    if (MIN_AUTORETRACT <= MAX_AUTORETRACT) {
      if (fwretract.autoretract_enabled && parser.seen_test('E')
        && !parser.seen(STR_AXES_MAIN)
      ) {
        const float echange = destination.e - current_position.e;
        if (WITHIN(ABS(echange), MIN_AUTORETRACT, MAX_AUTORETRACT) && fwretract.retracted[active_extruder] == (echange > 0.0)) {
          current_position.e = destination.e;
          sync_plan_position_e();
          return fwretract.retract(echange < 0.0);
        }
      }
    }
  #endif

  #if IS_SCARA
    fast_move ? prepare_fast_move_to_destination() : prepare_line_to_destination();
  #else
    prepare_line_to_destination();
  #endif

  #ifdef G0_FEEDRATE
    if (fast_move) feedrate_mm_s = old_feedrate;
  #endif

  #if ENABLED(NANODLP_Z_SYNC)
    #if ENABLED(NANODLP_ALL_AXIS)
      #define _MOVE_SYNC parser.seenval('X') || parser.seenval('Y') || parser.seenval('Z')
    #else
      #define _MOVE_SYNC parser.seenval('Z')
    #endif
    if (_MOVE_SYNC) {
      planner.synchronize();
      SERIAL_ECHOLNPGM(STR_Z_MOVE_COMP);
    }
    TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_IDLE));
  #else
    TERN_(FULL_REPORT_TO_HOST_FEATURE, report_current_grblstate_moving());
  #endif

  // ── CAN generic axis forwarding ───────────────────────────────────────
  //
  // Loop over every axis Marlin is configured for (X Y Z A B C …).
  // For each axis that appeared in this G0/G1 command, send a CAN frame
  // to the slave.
  //
  // destination[i] is already the absolute resolved target — G90/G91 was
  // applied by get_destination_from_command() above.
  //
  // ADDING A NEW AXIS: nothing changes here.  The LOOP_NUM_AXES macro
  // automatically covers all axes enabled in Configuration.h.
  //
  #if ENABLED(CAN_AXIS_SUPPORT)
    LOOP_NUM_AXES(i) {
      if (parser.seenval(AXIS_CHAR(i))) {
        can_axis_move((uint8_t)i, destination[i], axis_is_relative((AxisEnum)i));
      }
    }
  #endif
  // ─────────────────────────────────────────────────────────────────────
}
