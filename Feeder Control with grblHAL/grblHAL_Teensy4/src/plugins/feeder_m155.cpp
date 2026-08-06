// ============================================================
// feeder_m155.cpp  —  grblHAL plugin: M155 → CAN bus
//
// Syntax : M155 L<slaveId> D<motorId> S<rpm>
// Example: M155 L1 D0 S300   → slave 1, motor 0, 300 RPM
//          M155 L1 D0 S0     → stop motor 0 on slave 1
//          M155 L1 D0 S999   → emergency stop all on slave 1
//
// Place at: src/plugins/feeder_m155.cpp
// ============================================================

// FlexCAN_T4 must come first — it includes <algorithm> (C++ STL)
// Compiling as .c would fail; .cpp is required.
#include <FlexCAN_T4.h>

// grblHAL headers are C — extern "C" prevents C++ name-mangling
extern "C" {
  #include "driver.h"
  #include "grbl/hal.h"             // hal.stream.write()
  #include "grbl/core_handlers.h"   // user_mcode_ptrs_t, user_mcode_type_t
  #include "grbl/protocol.h"
  #include "grbl/nuts_bolts.h"
  #include "grbl/gcode.h"           // Feeder_M155, parser_block_t
}

#include "can_protocol.h"

// ── CAN1: TX=pin22 DEF, RX=pin23 DEF ────────────────────────
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> feeder_can;
static CAN_message_t tx_msg;

// ── Previous handler chain (for chaining plugins) ─────────────
static user_mcode_ptrs_t prev;

// ─────────────────────────────────────────────────────────────
// CAN helpers
// ─────────────────────────────────────────────────────────────

static void can_rx_handler(const CAN_message_t &msg)
{
    // Accept ACKs from any slave (0x201 .. 0x20F)
    if (msg.id < MOTOR_ACK_ID_BASE || msg.id > MOTOR_ACK_ID_BASE + 15)
        return;
    // Optional: add debug print here
}

// TX-complete handler — REQUIRED.
// enableFIFO(true) calls enableMBInterrupt() on every TX mailbox
// (see FlexCAN_T4.tpp enableFIFO()), which enables the TX-complete
// interrupt for each one. Without a registered onTransmit() handler,
// that interrupt fires into an empty dispatch slot and the mailbox
// is never reliably returned to TX_INACTIVE. write() only picks
// mailboxes whose code == TX_INACTIVE (FlexCAN_T4.tpp write(), the
// "for (uint8_t i = mailboxOffset(); ...)" loop), so each stuck
// mailbox permanently shrinks the usable TX pool — explaining a
// fixed small number of working commands followed by a hang that
// only a power cycle clears, regardless of which slave/feeder is
// targeted.
static void can_tx_handler(const CAN_message_t &msg)
{
    // No action needed — just servicing the interrupt is what frees
    // the mailbox back to TX_INACTIVE for reuse.
    (void)msg;
}

static void send_motor_command(uint8_t slave_id, uint8_t cmd,
                               uint8_t motor_id, uint16_t value)
{
    tx_msg.id     = MOTOR_CMD_ID_BASE + slave_id;
    tx_msg.buf[0] = cmd;
    tx_msg.buf[1] = motor_id;
    tx_msg.buf[2] = (uint8_t)(value & 0xFF);
    tx_msg.buf[3] = (uint8_t)(value >> 8);
    tx_msg.buf[4] = 0;
    tx_msg.buf[5] = 0;
    tx_msg.buf[6] = 0;
    tx_msg.buf[7] = 0;

    if (!feeder_can.write(tx_msg))
        hal.stream.write("[MSG:CAN TX FAILED]\r\n");
}

// ─────────────────────────────────────────────────────────────
// grblHAL M-code handler — three required functions
// ─────────────────────────────────────────────────────────────

// check(): called by parser to identify which plugin owns this M-code.
// Returns user_mcode_type_t — NOT user_mcode_t.
// UserMCode_Ignore does NOT exist; use UserMCode_Unsupported.
static user_mcode_type_t m155_check(user_mcode_t mcode)
{
    if (mcode == Feeder_M155)
        return UserMCode_Normal;

    return prev.check ? prev.check(mcode) : UserMCode_Unsupported;
}

// validate(): verify parameters and clear consumed word flags.
// Signature is ONE argument — confirmed from core_handlers.h.
// 
// Type notes from gc_values_t in gcode.h:
//   l  → uint32_t  (slave ID)   — check words.l flag, no isnan()
//   s  → float     (RPM)        — check words.s + isnan()
//   d  → float     (motor ID)   — optional; grblHAL sets NaN if absent
//
// IMPORTANT: clear words flags here. execute() must NOT use words.d
// to check presence — use isnan(values.d) instead, because words
// flags are already cleared by the time execute() runs.
static status_code_t m155_validate(parser_block_t *gc_block)
{
    if (gc_block->user_mcode != Feeder_M155)
        return prev.validate ? prev.validate(gc_block)
                             : Status_GcodeUnsupportedCommand;

    // L word (slave ID) is required
    if (!gc_block->words.l)
        return Status_GcodeValueWordMissing;

    // S word (RPM) is required
    if (!gc_block->words.s || isnan(gc_block->values.s))
        return Status_GcodeValueWordMissing;

    // D word (motor ID) is optional — NaN if absent, that's fine
    // Clear all consumed flags so grblHAL doesn't report unused words
    gc_block->words.l = Off;
    gc_block->words.s = Off;
    gc_block->words.d = Off;

    return Status_OK;
}

// execute(): send the CAN frame.
// Called after validate() — words flags are already cleared.
// Use isnan(values.d) to detect absent D word, NOT words.d.
static void m155_execute(sys_state_t state, parser_block_t *gc_block)
{
    if (gc_block->user_mcode != Feeder_M155) {
        if (prev.execute) prev.execute(state, gc_block);
        return;
    }

    uint8_t  slave_id = (uint8_t)gc_block->values.l;
    // D is optional — default motor 0 if not supplied (NaN check)
    uint8_t  motor_id = isnan(gc_block->values.d)
                        ? 0
                        : (uint8_t)gc_block->values.d;
    uint16_t rpm      = (uint16_t)gc_block->values.s;

    if (rpm == 999)
        send_motor_command(slave_id, CMD_ESTOP,       0,        0);
    else if (rpm == 0)
        send_motor_command(slave_id, CMD_STOP_MOTOR,  motor_id, 0);
    else
        send_motor_command(slave_id, CMD_START_MOTOR, motor_id, rpm);
}

// ─────────────────────────────────────────────────────────────
// Plugin entry point
// Name MUST be my_plugin_init — hardcoded in grbl/plugins_init.h
// extern "C" prevents C++ name-mangling so the C linker finds it
// ─────────────────────────────────────────────────────────────
extern "C" void my_plugin_init(void)
{
    // Initialise CAN1 (DEF pins: TX=22, RX=23)
    feeder_can.begin();
    feeder_can.setBaudRate(CAN_BAUDRATE);
    feeder_can.setMaxMB(16);
    feeder_can.enableFIFO();
    feeder_can.enableFIFOInterrupt();
    feeder_can.onReceive(can_rx_handler);
    feeder_can.onTransmit(can_tx_handler);
    feeder_can.mailboxStatus();

    tx_msg.flags.extended = 0;
    tx_msg.len = 8;

    hal.stream.write("[MSG:Feeder CAN ready 500kbps TX=22 RX=23]\r\n");

    // Chain onto grbl.user_mcode (in grbl_t, NOT hal)
    // Save existing handlers first so other plugins still work
    memcpy(&prev, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
    grbl.user_mcode.check    = m155_check;
    grbl.user_mcode.validate = m155_validate;
    grbl.user_mcode.execute  = m155_execute;
}