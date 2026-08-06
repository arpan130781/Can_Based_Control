#include <Arduino.h>
#include <eXoCAN.h>
#include "can_motion_protocol.h"

// ============================================================
// STM32F103 CAN Slave — Generic Multi-Axis + Feeder Firmware
//
// HOW TO ADD A NEW AXIS
// ─────────────────────
// 1. Wire your stepper driver to three STM32 pins.
// 2. Add ONE entry to axis_config[] below with those pins,
//    steps/mm, step delay, and the CAN_AXIS_* index.
// 3. Reflash the slave.
// That's it.  No changes needed on the Teensy / Marlin side.
//
// WIRING (all axes, same pattern):
//   STEP_PIN  : one rising edge = one microstep
//   DIR_PIN   : HIGH = positive direction
//   ENABLE_PIN: LOW = driver enabled (most drivers)
// ============================================================

// ── CAN bus ──────────────────────────────────────────────────
eXoCAN can;

const uint16_t CMD_ID_BASE = 0x100;
const uint16_t ACK_ID_BASE = 0x200;
const uint8_t  SLAVE_ID    = 1;

// ============================================================
// AXIS CONFIGURATION TABLE
// ─────────────────────────────────────────────────────────────
// Add, remove, or reorder entries freely.
// The axis_index field must match the CAN_AXIS_* constants in
// can_motion_protocol.h, which match Marlin's AxisEnum order.
// ============================================================

struct AxisConfig {
  uint8_t  axis_index;       // CAN_AXIS_X / _Y / _Z / _A / _B / _C …
  char     name;             // human-readable label for serial debug
  uint8_t  step_pin;
  uint8_t  dir_pin;
  uint8_t  enable_pin;       // LOW = driver on (most drivers)
  float    steps_per_mm;     // (motor_steps * microsteps) / lead_pitch_mm
  uint16_t step_delay_us;    // delay between steps; lower = faster
  uint16_t step_pulse_us;    // STEP pin HIGH time; ≥ 1 µs
};

// ── Axis table — edit this to match your hardware ────────────
//
// BUGFIX: the B-axis was originally wired to PC6/PC7/PC8. The Blue
// Pill (bluepill_f103c8) board only breaks out PC13/PC14/PC15 from
// GPIO port C on its physical headers — PC0-PC12 aren't connected to
// any pin on the board, so PlatformIO's board variant doesn't even
// declare those macros, causing a hard compile error ("'PC6' was not
// declared in this scope"). Re-routed B-axis to free pins instead:
// PA7 (unused — feeders only use PA0-PA6), PB10 (unused), and PC14
// (unused; safe as plain GPIO since Blue Pill ships without a 32kHz
// LSE crystal populated).
//
//  index        name  STEP   DIR    EN      st/mm  dly  pulse
static const AxisConfig axis_config[] = {
  { CAN_AXIS_Z,  'Z',  PA9,  PA10,  PB11,   800,   50,  2 },
  { CAN_AXIS_A,  'A',  PB0,  PB1,   PB2,    500,   50,  2 },
  { CAN_AXIS_B,  'B',  PA7,  PB10,  PC14,   500,   50,  2 },

  // To add axis C later:
  // { CAN_AXIS_C, 'C',  PA0,  PA1,   PA2,   400,  50,  2 },
};

static const uint8_t NUM_AXIS_CFG = sizeof(axis_config) / sizeof(axis_config[0]);

// Per-axis runtime position (mm), indexed parallel to axis_config[]
static float axis_position_mm[NUM_AXIS_CFG] = {0};

// ── Feeder pins ───────────────────────────────────────────────
// NOTE: Pins used by stepper axes are EXCLUDED:
//   Z-axis  : PA9,PA10,PB11
//   A-axis  : PB0,PB1,PB2
//   B-axis  : PA7,PB10,PC14
// CAN bus   : PA11(RX),PA12(TX) — never use as GPIO
#define NUMBER_OF_FEEDERS 20
const uint8_t feederPins[NUMBER_OF_FEEDERS] = {
  PB12, PB13, PB14, PB15,        // Feeders 0–3
  PA8,  PA15, PB3,  PB4,         // Feeders 4–7
  PB5,  PB6,  PB7,  PB8,         // Feeders 8–11
  PB9,  PA0,  PA1,  PA2,         // Feeders 12–15
  PA3,  PA4,  PA5,  PA6,         // Feeders 16–19
};
uint16_t currentRpm[NUMBER_OF_FEEDERS] = {0};

// ── RPM → PWM calibration table ──────────────────────────────
struct RpmPoint { uint16_t rpm; uint8_t pwm; };
static const RpmPoint rpmTable[] = {
  {   0,   0 }, {  50,  20 }, { 100,  40 }, { 200,  80 },
  { 300, 120 }, { 400, 160 }, { 500, 200 }, { 600, 230 }, { 700, 255 },
};
static const uint8_t RPM_TABLE_SIZE = sizeof(rpmTable) / sizeof(rpmTable[0]);

uint8_t rpmToPwm(uint16_t rpm) {
  if (rpm == 0) return 0;
  if (rpm <= rpmTable[1].rpm) return rpmTable[1].pwm;
  if (rpm >= rpmTable[RPM_TABLE_SIZE-1].rpm) return rpmTable[RPM_TABLE_SIZE-1].pwm;
  for (uint8_t i = 1; i < RPM_TABLE_SIZE-1; i++) {
    if (rpm <= rpmTable[i+1].rpm) {
      return (uint8_t)((uint16_t)rpmTable[i].pwm +
        ((uint32_t)(rpm - rpmTable[i].rpm) * (rpmTable[i+1].pwm - rpmTable[i].pwm))
        / (rpmTable[i+1].rpm - rpmTable[i].rpm));
    }
  }
  return 255;
}

// ── CAN RX buffers ────────────────────────────────────────────
int     rxId;
int     fltIdx;
uint8_t rxData[8];

#define LED_PIN PC13

uint32_t lastHeartbeat = 0;
uint32_t frameCount    = 0;
uint32_t droppedCount  = 0;

// ── Forward declarations ─────────────────────────────────────
void processMessage(int id, uint8_t *data);
void sendAck(uint8_t cmd, uint8_t arg1, uint8_t arg2);
void printStatus();

// Axis motion
int8_t  findAxis(uint8_t axis_index);
void    axisMoveTo(uint8_t cfg_idx, float target_mm);
void    axisStep(uint8_t cfg_idx, int32_t steps);
void    axisSetHome(uint8_t cfg_idx);

// Feeder
void setFeeder(uint8_t feeder, uint16_t rpm);
void stopFeeder(uint8_t feeder);
void setFeederSpeed(uint8_t feeder, uint16_t rpm);
void setFeederDirection(uint8_t feeder, uint8_t dir);
void emergencyStop();

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("\n============================================================"));
  Serial.println(F("  STM32F103 CAN Slave — Generic Multi-Axis + Feeder"));
  Serial.println(F("============================================================"));
  Serial.print(F("  Slave ID  : ")); Serial.println(SLAVE_ID);
  Serial.print(F("  CMD ID    : 0x")); Serial.println(CMD_ID_BASE + SLAVE_ID, HEX);
  Serial.print(F("  ACK ID    : 0x")); Serial.println(ACK_ID_BASE + SLAVE_ID, HEX);

  // Print axis table
  Serial.println(F("  Configured axes:"));
  for (uint8_t i = 0; i < NUM_AXIS_CFG; i++) {
    const AxisConfig &a = axis_config[i];
    Serial.print(F("    [")); Serial.print(i); Serial.print(F("] "));
    Serial.print((char)a.name);
    Serial.print(F("  STEP=")); Serial.print(a.step_pin);
    Serial.print(F("  DIR="));  Serial.print(a.dir_pin);
    Serial.print(F("  EN="));   Serial.print(a.enable_pin);
    Serial.print(F("  st/mm=")); Serial.println(a.steps_per_mm);
  }
  Serial.println(F("============================================================\n"));

  // Init feeder pins
  for (uint8_t i = 0; i < NUMBER_OF_FEEDERS; i++) {
    pinMode(feederPins[i], OUTPUT);
    digitalWrite(feederPins[i], LOW);
  }

  // Init axis stepper pins
  for (uint8_t i = 0; i < NUM_AXIS_CFG; i++) {
    const AxisConfig &a = axis_config[i];
    pinMode(a.step_pin,   OUTPUT); digitalWrite(a.step_pin,   LOW);
    pinMode(a.dir_pin,    OUTPUT); digitalWrite(a.dir_pin,    LOW);
    pinMode(a.enable_pin, OUTPUT); digitalWrite(a.enable_pin, HIGH); // disabled
  }

  can.begin(STD_ID_LEN, BR500K, PORTA_11_12_XCVR);
  can.filterMask16Init(0, 0, 0x000, 0, 0);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println(F("  Ready. Waiting for commands...\n"));
}

// ============================================================
// Main Loop
// ============================================================

void loop() {
  if (can.receive(rxId, fltIdx, rxData) > -1) {
    frameCount++;
    digitalToggle(LED_PIN);

    Serial.print(F("[RX] 0x")); Serial.print(rxId, HEX);
    Serial.print(F(" | "));
    for (int i = 0; i < 8; i++) {
      if (rxData[i] < 0x10) Serial.print('0');
      Serial.print(rxData[i], HEX); Serial.print(' ');
    }

    if (rxId == CMD_ID_BASE + SLAVE_ID) {
      Serial.println(F(" ← MINE"));
      processMessage(rxId, rxData);
    } else {
      droppedCount++;
      Serial.println(F(" ← ignored"));
    }
  }

  if (millis() - lastHeartbeat >= 5000) {
    lastHeartbeat = millis();
    printStatus();
  }
}

// ============================================================
// Status heartbeat
// ============================================================

void printStatus() {
  Serial.println(F("\n--- HEARTBEAT ---"));
  Serial.print(F("  Frames rx : ")); Serial.println(frameCount);
  Serial.print(F("  Dropped   : ")); Serial.println(droppedCount);
  for (uint8_t i = 0; i < NUM_AXIS_CFG; i++) {
    Serial.print(F("  Axis "));
    Serial.print((char)axis_config[i].name);
    Serial.print(F("       : "));
    Serial.print(axis_position_mm[i]);
    Serial.println(F(" mm"));
  }
  bool any = false;
  Serial.print(F("  Feeders   : "));
  for (uint8_t i = 0; i < NUMBER_OF_FEEDERS; i++) {
    if (currentRpm[i] > 0) {
      Serial.print(F("F")); Serial.print(i);
      Serial.print('='); Serial.print(currentRpm[i]); Serial.print(F("rpm "));
      any = true;
    }
  }
  if (!any) Serial.print(F("all OFF"));
  Serial.println(F("\n-----------------\n"));
}

// ============================================================
// Process incoming CAN message
// ============================================================

void processMessage(int id, uint8_t *data) {
  uint8_t cmd = data[0];

  Serial.println(F("\n============================================================"));
  Serial.print(F("  CMD: 0x")); Serial.print(cmd, HEX);

  switch (cmd) {

    // ── Generic axis move ─────────────────────────────────────────────
    case CMD_AXIS_MOVE: {
      uint8_t axis_idx = data[1];
      bool    relative = (data[2] & 0x01) != 0;
      union { float f; uint8_t b[4]; } u;
      u.b[0]=data[3]; u.b[1]=data[4]; u.b[2]=data[5]; u.b[3]=data[6];
      float target = u.f;

      Serial.println(F(" (AXIS_MOVE)"));
      Serial.print(F("  axis_idx=")); Serial.print(axis_idx);
      Serial.print(F("  mode="));    Serial.print(relative ? F("REL") : F("ABS"));
      Serial.print(F("  target="));  Serial.print(target); Serial.println(F(" mm"));

      int8_t cfg = findAxis(axis_idx);
      if (cfg < 0) {
        Serial.print(F("  ERROR: axis ")); Serial.print(axis_idx);
        Serial.println(F(" not in axis_config[] — add it and reflash slave"));
      } else {
        axisMoveTo((uint8_t)cfg, target);
        sendAck(CMD_AXIS_MOVE, axis_idx, 0);
      }
      break;
    }

    // ── Generic axis home ─────────────────────────────────────────────
    case CMD_AXIS_HOME: {
      uint8_t axis_idx = data[1];

      Serial.println(F(" (AXIS_HOME)"));
      Serial.print(F("  axis_idx=")); Serial.println(axis_idx);

      int8_t cfg = findAxis(axis_idx);
      if (cfg < 0) {
        Serial.print(F("  ERROR: axis ")); Serial.print(axis_idx);
        Serial.println(F(" not configured on slave"));
      } else {
        axisSetHome((uint8_t)cfg);
        sendAck(CMD_AXIS_HOME, axis_idx, 0);
      }
      break;
    }

    // ── Feeder commands ───────────────────────────────────────────────
    case CMD_START_MOTOR: {
      Serial.println(F(" (START_MOTOR)"));
      uint8_t  feeder = data[1];
      uint16_t rpm    = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
      setFeeder(feeder, rpm);
      break;
    }
    case CMD_STOP_MOTOR: {
      Serial.println(F(" (STOP_MOTOR)"));
      stopFeeder(data[1]);
      break;
    }
    case CMD_SET_SPEED: {
      Serial.println(F(" (SET_SPEED)"));
      uint16_t rpm = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
      setFeederSpeed(data[1], rpm);
      break;
    }
    case CMD_SET_DIRECTION: {
      Serial.println(F(" (SET_DIRECTION)"));
      setFeederDirection(data[1], data[2]);
      break;
    }
    case CMD_ESTOP: {
      Serial.println(F(" (ESTOP)"));
      emergencyStop();
      break;
    }
    case CMD_PING: {
      Serial.println(F(" (PING)"));
      sendAck(CMD_PING, 0, 0);
      break;
    }
    default:
      Serial.print(F(" (UNKNOWN cmd=0x")); Serial.print(cmd, HEX); Serial.println(F(")"));
      break;
  }
}

// ============================================================
// Axis helpers
// ============================================================

// Find the axis_config[] index for a given CAN axis index.
// Returns -1 if not configured on this slave — slave just logs the error.
int8_t findAxis(uint8_t axis_index) {
  for (uint8_t i = 0; i < NUM_AXIS_CFG; i++) {
    if (axis_config[i].axis_index == axis_index) return (int8_t)i;
  }
  return -1;
}

// Move axis cfg_idx to absolute position target_mm
void axisMoveTo(uint8_t cfg_idx, float target_mm) {
  const AxisConfig &a = axis_config[cfg_idx];
  float   delta = target_mm - axis_position_mm[cfg_idx];
  int32_t steps = (int32_t)(delta * a.steps_per_mm);

  if (steps == 0) {
    Serial.print(F("  Axis ")); Serial.print((char)a.name);
    Serial.println(F(": already at target"));
    return;
  }

  Serial.print(F("  Axis ")); Serial.print((char)a.name);
  Serial.print(F(": ")); Serial.print(axis_position_mm[cfg_idx]);
  Serial.print(F(" -> ")); Serial.print(target_mm);
  Serial.print(F(" mm (")); Serial.print(steps); Serial.println(F(" steps)"));

  axisStep(cfg_idx, steps);
  axis_position_mm[cfg_idx] = target_mm;

  // Disable driver after move (save power).
  // Remove if you need holding torque between moves.
  digitalWrite(a.enable_pin, HIGH);

  Serial.print(F("  Axis ")); Serial.print((char)a.name);
  Serial.print(F(": done @ ")); Serial.print(target_mm); Serial.println(F(" mm"));
  Serial.println(F("============================================================\n"));
}

// Pulse the stepper `steps` microsteps (sign = direction)
void axisStep(uint8_t cfg_idx, int32_t steps) {
  const AxisConfig &a = axis_config[cfg_idx];

  digitalWrite(a.enable_pin, LOW);   // enable driver
  delayMicroseconds(5);

  if (steps > 0) {
    digitalWrite(a.dir_pin, HIGH);
  } else {
    digitalWrite(a.dir_pin, LOW);
    steps = -steps;
  }
  delayMicroseconds(2);              // DIR setup time

  for (int32_t i = 0; i < steps; i++) {
    digitalWrite(a.step_pin, HIGH);
    delayMicroseconds(a.step_pulse_us);
    digitalWrite(a.step_pin, LOW);
    delayMicroseconds(a.step_delay_us);
  }
}

// Reset position counter to 0 (called after G28 homing — don't move motor)
void axisSetHome(uint8_t cfg_idx) {
  const AxisConfig &a = axis_config[cfg_idx];
  axis_position_mm[cfg_idx] = 0.0f;
  digitalWrite(a.enable_pin, HIGH);  // disable driver
  Serial.print(F("  Axis ")); Serial.print((char)a.name);
  Serial.println(F(": HOME — position reset to 0.00 mm"));
  Serial.println(F("============================================================\n"));
}

// ============================================================
// Feeder control
// ============================================================

void setFeeder(uint8_t f, uint16_t rpm) {
  if (f >= NUMBER_OF_FEEDERS) { Serial.println(F("  ERR: bad feeder")); return; }
  currentRpm[f] = rpm;
  uint8_t pwm = rpmToPwm(rpm);
  if (rpm == 0)      digitalWrite(feederPins[f], LOW);
  else if (pwm==255) digitalWrite(feederPins[f], HIGH);
  else               analogWrite(feederPins[f], pwm);
  Serial.print(F("  Feeder ")); Serial.print(f);
  Serial.print(F(" -> ")); Serial.print(rpm); Serial.println(F(" RPM"));
  sendAck(CMD_START_MOTOR, f, pwm);
}

void stopFeeder(uint8_t f) {
  if (f >= NUMBER_OF_FEEDERS) return;
  currentRpm[f] = 0;
  digitalWrite(feederPins[f], LOW);
  Serial.print(F("  Feeder ")); Serial.print(f); Serial.println(F(" STOPPED"));
  sendAck(CMD_STOP_MOTOR, f, 0);
}

void setFeederSpeed(uint8_t f, uint16_t rpm) {
  if (f >= NUMBER_OF_FEEDERS) return;
  currentRpm[f] = rpm;
  analogWrite(feederPins[f], rpmToPwm(rpm));
  sendAck(CMD_SET_SPEED, f, rpmToPwm(rpm));
}

void setFeederDirection(uint8_t f, uint8_t dir) {
  if (f >= NUMBER_OF_FEEDERS) return;
  Serial.print(F("  Feeder ")); Serial.print(f);
  Serial.print(F(" DIR=")); Serial.println(dir);
  sendAck(CMD_SET_DIRECTION, f, dir);
}

void emergencyStop() {
  Serial.println(F("\n  !!! ESTOP: all feeders OFF, all axes disabled !!!"));
  for (uint8_t i = 0; i < NUMBER_OF_FEEDERS; i++) {
    digitalWrite(feederPins[i], LOW); currentRpm[i] = 0;
  }
  for (uint8_t i = 0; i < NUM_AXIS_CFG; i++) {
    digitalWrite(axis_config[i].enable_pin, HIGH);
  }
}

// ============================================================
// ACK
// ============================================================

void sendAck(uint8_t cmd, uint8_t arg1, uint8_t arg2) {
  uint8_t d[3] = {cmd, arg1, arg2};
  bool ok = can.transmit(ACK_ID_BASE + SLAVE_ID, d, 3);
  Serial.print(F("  ACK 0x")); Serial.print(ACK_ID_BASE + SLAVE_ID, HEX);
  Serial.print(F(" cmd=0x")); Serial.print(cmd, HEX);
  Serial.println(ok ? F(" [OK]") : F(" [FAIL]"));
}
