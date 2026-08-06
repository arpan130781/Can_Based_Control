// ============================================================
// boards/my_machine_map.h
// Custom pin map for Teensy 4.1 Feeder CAN Master
//
// Changes from generic_map.h:
//   1. Z_LIMIT_PIN moved from pin 22 → pin 33  (22 = CAN1 TX)
//   2. RESET_PIN, FEED_HOLD_PIN, CYCLE_START_PIN removed entirely
//      — these float and trigger ALARM:10 (EStop) on boot
//      — this machine has no physical control switches
// ============================================================

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

// ── Stepper outputs (not used but required by grblHAL) ───────
#define X_STEP_PIN          (2u)
#define Y_STEP_PIN          (4u)
#define Z_STEP_PIN          (6u)
#define X_DIRECTION_PIN     (3u)
#define Y_DIRECTION_PIN     (5u)
#define Z_DIRECTION_PIN     (7u)
#define STEPPERS_ENABLE_PIN (10u)

// ── Limit inputs ──────────────────────────────────────────────
// Z_LIMIT moved from 22 → 33 to free CAN1 TX (pin 22)
#define X_LIMIT_PIN         (20u)
#define Y_LIMIT_PIN         (21u)
#define Z_LIMIT_PIN         (33u)

// ── Auxiliary outputs ─────────────────────────────────────────
#define AUXOUTPUT0_PIN      (13u)
#define AUXOUTPUT1_PIN      (11u)
#define AUXOUTPUT2_PIN      (12u)
#define AUXOUTPUT3_PIN      (19u)
#define AUXOUTPUT4_PIN      (18u)

#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

// ── Auxiliary inputs ──────────────────────────────────────────
#define AUXINPUT0_PIN       (0u)
#define AUXINPUT1_PIN       (3u)
#define AUXINPUT2_PIN       (1u)
#define AUXINPUT3_PIN       (29u)
#define AUXINPUT4_PIN       (15u)
// AUXINPUT5 (14), AUXINPUT6 (16), AUXINPUT7 (17) intentionally
// NOT defined — these were mapped to RESET/FEED_HOLD/CYCLE_START
// and float-triggered control alarms.

// ── Control inputs — NOT defined (no hardware connected) ──────
// Defining RESET_PIN with ESTOP_ENABLE=1 (default) reads it as
// e_stop. Floating pin → ALARM:10 on every boot.
// RESET_PIN, FEED_HOLD_PIN, CYCLE_START_PIN are all omitted.

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT4_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     AUXINPUT3_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN     AUXINPUT3_PIN
#endif

#if I2C_ENABLE
#define I2C_PORT            4
#define I2C_SCL4            (24u)
#define I2C_SDA4            (25u)
#endif

#if ENCODER_ENABLE
#define QEI_A_PIN           AUXINPUT0_PIN
#define QEI_B_PIN           AUXINPUT1_PIN
#if (ENCODER_ENABLE & 1) && defined(AUXINPUT2_PIN)
#define QEI_SELECT_PIN      AUXINPUT2_PIN
#endif
#endif