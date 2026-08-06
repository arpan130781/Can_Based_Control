#pragma once

// ── Board map ─────────────────────────────────────────────────
// Uses src/boards/my_machine_map.h instead of generic_map.h.
// generic_map.h assigns Z_LIMIT_PIN=22 (= CAN1 TX on this board).
#define BOARD_MY_MACHINE

// ── Plugin ────────────────────────────────────────────────────
#define ADD_MY_PLUGIN       1

// ── Serial ───────────────────────────────────────────────────
#define USB_SERIAL_CDC      1

// ── Disable unused inputs ─────────────────────────────────────
// No probe, no physical E-stop, no limit switches connected.
// ESTOP_ENABLE=1 (the default when COMPATIBILITY_LEVEL<=1) causes
// driver.c to read RESET_PIN (pin 14) as e_stop. Pin 14 floats
// high → triggers ALARM:10 on every boot.
#define PROBE_ENABLE        0
#define ESTOP_ENABLE        0

// ── DO NOT define N_SPINDLE 0 ────────────────────────────────
// Minimum allowed by config.h is 1. Setting 0 → ALARM:16.