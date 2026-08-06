#pragma once
/**
 * custom/M155.h
 *
 * Declares the GcodeSuite::M155() method that implements the custom
 *   M155 L<led> D<dir> S<speed>
 * command.  This header is #included by gcode.cpp immediately after
 * the existing custom/M755.h include so the translation unit sees the
 * declaration before the dispatch switch.
 *
 * Guard: compiled only when CUSTOM_M155_COMMAND is defined.
 */

#include "../gcode.h"   // pulls in the GcodeSuite class declaration

#if ENABLED(CUSTOM_M155_COMMAND)
  // The static method is declared inside GcodeSuite in gcode.h (see the
  // #if ENABLED(CUSTOM_M155_COMMAND) block added there).  This header
  // exists so gcode.cpp can do a single, self-documenting include rather
  // than relying on gcode.h alone, matching the convention set by M755.h.
  void M155();  // implemented in custom/M155.cpp
#endif