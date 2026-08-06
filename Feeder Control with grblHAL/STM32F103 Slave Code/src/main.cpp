#include <Arduino.h>
#include <eXoCAN.h>

// ============================================================
// CAN Configuration — STM32F103 Blue Pill
// PA11 = CAN RX,  PA12 = CAN TX,  via MCP2551 transceiver
// ============================================================

eXoCAN can;

// ── Command bytes (mirrored from Teensy master) ──────────────
#define CMD_START_MOTOR   0x10
#define CMD_STOP_MOTOR    0x11
#define CMD_SET_SPEED     0x12
#define CMD_SET_DIRECTION 0x13
#define CMD_ESTOP         0x14
#define CMD_PING          0x15
#define CMD_ACK           0x16

const uint16_t CMD_ID_BASE = 0x100;   // master sends to 0x100 + SLAVE_ID
const uint16_t ACK_ID_BASE = 0x200;   // slave replies  on 0x200 + SLAVE_ID
const uint8_t  SLAVE_ID    = 1;       // this board's ID — must match L word

// ── Output pins — 27 feeders ──────────────────────────────────
#define NUMBER_OF_FEEDERS 27
#define NUM_PINS NUMBER_OF_FEEDERS   // kept as alias so rest of file is unchanged

const uint8_t feederPins[NUMBER_OF_FEEDERS] = {
    PB12, PB13, PB14, PB15, PA8, PA15, PB3, PB4, PB5, PB6, PB7, PB8,
    PB9, PC13, PC14, PC15, PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
    PB0, PB1, PB10
};

// outputPins kept as the name used elsewhere in the file
#define outputPins feederPins

uint16_t currentValues[NUM_PINS] = {0};

// ── CAN receive buffers ──────────────────────────────────────
int     rxId;
int     fltIdx;
uint8_t rxData[8];

// ── Status LED (active LOW on Blue Pill) ─────────────────────
// NOTE: PC13 is also used as feeder index 13 in feederPins[] above.
// PC13 is the onboard LED on most Blue Pill boards, so driving it as a
// feeder output will also toggle/override the heartbeat LED, and the
// heartbeat blink-on-RX will affect feeder 13's output state.
// If you want a dedicated, conflict-free heartbeat LED, move it to a
// pin NOT present in feederPins[] (e.g. PA9) instead.
#define LED PC13

// ── Heartbeat ────────────────────────────────────────────────
uint32_t lastHeartbeat  = 0;
uint32_t lastCanFrame   = 0;
uint32_t frameCount     = 0;
uint32_t droppedCount   = 0;   // frames received but not for this slave

// ── Forward declarations ─────────────────────────────────────
void processMessage(int id, uint8_t *data);
void setFeeder(uint8_t feeder, uint16_t rpm);
void stopFeeder(uint8_t feeder);
void setFeederSpeed(uint8_t feeder, uint16_t rpm);
void setFeederDirection(uint8_t feeder, uint8_t direction);
void emergencyStop();
void sendAck(uint8_t cmd, uint8_t feeder, uint8_t value);
void printStatus();

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(200);

    Serial.println(F("\n============================================================"));
    Serial.println(F("  STM32F103 CAN Feeder Slave — DEBUG BUILD"));
    Serial.println(F("============================================================"));
    Serial.print(F("  Slave ID     : ")); Serial.println(SLAVE_ID);
    Serial.print(F("  Listens on   : 0x")); Serial.println(CMD_ID_BASE + SLAVE_ID, HEX);
    Serial.print(F("  ACK reply on : 0x")); Serial.println(ACK_ID_BASE + SLAVE_ID, HEX);
    Serial.print(F("  CAN speed    : 500 kbps"));
    Serial.println(F("  Pins         : PA11(RX) PA12(TX) MCP2551"));
    Serial.println(F("============================================================\n"));

    // Configure output pins
    for (int i = 0; i < NUM_PINS; i++)
    {
        pinMode(outputPins[i], OUTPUT);
        digitalWrite(outputPins[i], LOW);
        Serial.print(F("  Feeder ")); Serial.print(i); Serial.println(F(" = OUTPUT (feeder off)"));
    }
    Serial.println();

    // Init CAN
    can.begin(STD_ID_LEN, BR500K, PORTA_11_12_XCVR);
    // Accept ALL IDs so we can see if wrong-ID frames arrive
    // Filter: base=0, mask=0 → pass everything
    can.filterMask16Init(0, 0, 0x000, 0, 0);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);   // LED off (active LOW)

    Serial.println(F("  CAN bus ready — filter accepts ALL IDs for diagnostics"));
    Serial.println(F("  Heartbeat every 5s | Waiting for M155 command...\n"));
    Serial.println(F("------------------------------------------------------------"));
}

// ============================================================
// Main Loop
// ============================================================

void loop()
{
    // ── CAN receive ──────────────────────────────────────────
    if (can.receive(rxId, fltIdx, rxData) > -1)
    {
        frameCount++;
        lastCanFrame = millis();
        digitalToggle(LED);   // blink on any CAN frame

        Serial.print(F("[CAN RX] ID=0x"));
        Serial.print(rxId, HEX);
        Serial.print(F(" Data: "));
        for (int i = 0; i < 8; i++) {
            if (rxData[i] < 0x10) Serial.print('0');
            Serial.print(rxData[i], HEX);
            Serial.print(' ');
        }

        if (rxId == CMD_ID_BASE + SLAVE_ID)
        {
            Serial.println(F(" ← MINE"));
            processMessage(rxId, rxData);
        }
        else
        {
            droppedCount++;
            Serial.print(F(" ← NOT FOR ME (expect 0x"));
            Serial.print(CMD_ID_BASE + SLAVE_ID, HEX);
            Serial.println(F(") — wrong slave ID?"));
        }
    }

    // ── Heartbeat every 5 seconds ────────────────────────────
    if (millis() - lastHeartbeat >= 5000)
    {
        lastHeartbeat = millis();
        printStatus();
    }
}

// ============================================================
// Status print — called every 5 s
// ============================================================

void printStatus()
{
    Serial.println(F("\n--- HEARTBEAT ---"));
    Serial.print(F("  Uptime      : ")); Serial.print(millis() / 1000); Serial.println(F(" s"));
    Serial.print(F("  CAN frames  : ")); Serial.println(frameCount);
    Serial.print(F("  Wrong-ID    : ")); Serial.println(droppedCount);

    if (lastCanFrame == 0)
        Serial.println(F("  Last frame  : none received yet"));
    else {
        Serial.print(F("  Last frame  : ")); Serial.print((millis() - lastCanFrame) / 1000);
        Serial.println(F(" s ago"));
    }

    Serial.print(F("  Feeders     : "));
    bool any = false;
    for (int i = 0; i < NUM_PINS; i++) {
        if (currentValues[i] > 0) {
            Serial.print(F("F")); Serial.print(i);
            Serial.print('='); Serial.print(currentValues[i]);
            Serial.print(' ');
            any = true;
        }
    }
    if (!any) Serial.print(F("all OFF"));
    Serial.println();
    Serial.println(F("-----------------\n"));
}

// ============================================================
// Process CAN Message
// ============================================================

void processMessage(int id, uint8_t *data)
{
    uint8_t  cmd    = data[0];
    uint8_t  feeder = data[1];
    uint16_t rpm    = (uint16_t)data[2] | ((uint16_t)data[3] << 8);

    Serial.println(F("\n============================================================"));
    Serial.println(F("  G-CODE COMMAND RECEIVED & PARSED"));
    Serial.println(F("============================================================"));
    Serial.print(F("  CAN ID  : 0x")); Serial.println(id, HEX);
    Serial.print(F("  CMD     : 0x")); Serial.print(cmd, HEX);
    switch(cmd) {
        case CMD_START_MOTOR:   Serial.println(F(" (START_MOTOR)"));     break;
        case CMD_STOP_MOTOR:    Serial.println(F(" (STOP_MOTOR)"));      break;
        case CMD_SET_SPEED:     Serial.println(F(" (SET_SPEED)"));       break;
        case CMD_SET_DIRECTION: Serial.println(F(" (SET_DIRECTION)"));   break;
        case CMD_ESTOP:         Serial.println(F(" (ESTOP)"));           break;
        case CMD_PING:          Serial.println(F(" (PING)"));            break;
        default:                Serial.println(F(" (UNKNOWN)"));         break;
    }
    Serial.print(F("  Feeder  : ")); Serial.println(feeder);
    Serial.print(F("  RPM/Val : ")); Serial.println(rpm);
    Serial.print(F("  Raw     : "));
    for (int i = 0; i < 8; i++) {
        if (data[i] < 0x10) Serial.print('0');
        Serial.print(data[i], HEX); Serial.print(' ');
    }
    Serial.println();
    Serial.println(F("------------------------------------------------------------"));

    switch (cmd)
    {
        case CMD_START_MOTOR:   setFeeder(feeder, rpm);            break;
        case CMD_STOP_MOTOR:    stopFeeder(feeder);                break;
        case CMD_SET_SPEED:     setFeederSpeed(feeder, rpm);       break;
        case CMD_SET_DIRECTION: setFeederDirection(feeder,data[2]);break;
        case CMD_ESTOP:         emergencyStop();                   break;
        case CMD_PING:
            Serial.println(F("  PING → sending PONG ACK"));
            sendAck(CMD_PING, 0, 0);
            break;
        default:
            Serial.print(F("  ERROR: Unknown command 0x"));
            Serial.println(cmd, HEX);
            break;
    }
}

// ============================================================
// Feeder control functions
// ============================================================

void setFeeder(uint8_t feeder, uint16_t rpm)
{
    if (feeder >= NUM_PINS) {
        Serial.print(F("  ERROR: Invalid feeder ")); Serial.println(feeder);
        return;
    }
    currentValues[feeder] = rpm;

    if (rpm == 0) {
        digitalWrite(outputPins[feeder], LOW);
        Serial.print(F("  RESULT: Feeder ")); Serial.print(feeder); Serial.println(F(" → OFF"));
    } else if (rpm >= 255) {
        digitalWrite(outputPins[feeder], HIGH);
        Serial.print(F("  RESULT: Feeder ")); Serial.print(feeder); Serial.println(F(" → ON FULL (digital HIGH)"));
    } else {
        analogWrite(outputPins[feeder], (uint8_t)rpm);
        Serial.print(F("  RESULT: Feeder ")); Serial.print(feeder);
        Serial.print(F(" → PWM=")); Serial.println(rpm);
    }
    sendAck(CMD_START_MOTOR, feeder, (uint8_t)(rpm > 255 ? 255 : rpm));
}

void stopFeeder(uint8_t feeder)
{
    if (feeder >= NUM_PINS) {
        Serial.print(F("  ERROR: Invalid feeder ")); Serial.println(feeder);
        return;
    }
    currentValues[feeder] = 0;
    digitalWrite(outputPins[feeder], LOW);
    Serial.print(F("  RESULT: Feeder ")); Serial.print(feeder); Serial.println(F(" → STOPPED"));
    sendAck(CMD_STOP_MOTOR, feeder, 0);
}

void setFeederSpeed(uint8_t feeder, uint16_t rpm)
{
    if (feeder >= NUM_PINS) {
        Serial.print(F("  ERROR: Invalid feeder ")); Serial.println(feeder);
        return;
    }
    currentValues[feeder] = rpm;
    analogWrite(outputPins[feeder], (uint8_t)(rpm > 255 ? 255 : rpm));
    Serial.print(F("  RESULT: Feeder ")); Serial.print(feeder);
    Serial.print(F(" → Speed=")); Serial.println(rpm);
    sendAck(CMD_SET_SPEED, feeder, (uint8_t)(rpm > 255 ? 255 : rpm));
}

void setFeederDirection(uint8_t feeder, uint8_t direction)
{
    if (feeder >= NUM_PINS) {
        Serial.print(F("  ERROR: Invalid feeder ")); Serial.println(feeder);
        return;
    }
    Serial.print(F("  RESULT: Feeder ")); Serial.print(feeder);
    Serial.print(F(" → Direction=")); Serial.println(direction ? F("REVERSE") : F("FORWARD"));
    sendAck(CMD_SET_DIRECTION, feeder, direction);
}

void emergencyStop()
{
    Serial.println(F("\n  !!! EMERGENCY STOP — all feeders OFF !!!"));
    for (int i = 0; i < NUM_PINS; i++) {
        digitalWrite(outputPins[i], LOW);
        currentValues[i] = 0;
        Serial.print(F("  Feeder ")); Serial.print(i); Serial.println(F(" OFF"));
    }
    Serial.println();
}

// ============================================================
// Send ACK back to Teensy master
// ============================================================

void sendAck(uint8_t cmd, uint8_t feeder, uint8_t value)
{
    uint8_t ackData[3] = {cmd, feeder, value};
    bool ok = can.transmit(ACK_ID_BASE + SLAVE_ID, ackData, 3);

    Serial.print(F("  ACK → ID=0x")); Serial.print(ACK_ID_BASE + SLAVE_ID, HEX);
    Serial.print(F(" CMD=0x")); Serial.print(cmd, HEX);
    Serial.print(F(" Feeder=")); Serial.print(feeder);
    Serial.print(F(" Val=")); Serial.print(value);
    Serial.println(ok ? F(" [SENT OK]") : F(" [SEND FAILED]"));
    Serial.println(F("============================================================\n"));
}