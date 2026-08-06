# CAN_Based_Control

**Design and Development of a CAN-Based Distributed Motion Control System**
From grblHAL feeder control to a Marlin-integrated CAN motion control framework.

A Teensy 4.1 master (running **grblHAL** or **Marlin**) communicates over a 500 kbps CAN bus with one or more **STM32F103 "Blue Pill"** slave controllers, relaying G-code driven commands to drive steppers, feeders, and solenoids — distributed I/O that can't all be wired directly to the master. This repository holds three firmware builds tracing that system's evolution, from a fixed feeder-control protocol to a general-purpose CAN-to-G-code motion bridge.

Originally developed during an embedded systems internship at **Invariance Automation Pvt. Ltd.** (incubated at IIT Kanpur), 22 May – 27 June 2026.

---

## Repository Structure

```
CAN_Based_Control/
├── Feeder_Control_with_grblHAL/     # Teensy 4.1 (grblHAL) master + STM32 feeder slave
├── Feeder_Control_with_Marlin/      # Teensy 4.1 (Marlin) master + STM32 feeder slave
└── Motor_Control_with_Marlin/       # Teensy 4.1 (Marlin) master + STM32 axis + feeder slave
```

| Sub-project | Master firmware | Slave capability | Role in the project's evolution |
|---|---|---|---|
| `Feeder_Control_with_grblHAL` | grblHAL | Fixed feeder commands | Original architecture; where the core CAN reception bug was found and fixed |
| `Feeder_Control_with_Marlin` | Marlin | Fixed feeder commands | Ported implementation, physically validated with solenoid control |
| `Motor_Control_with_Marlin` | Marlin | RPM-based motor control + local G-code engine | Final, generalized CAN-to-G-code motion bridge |

---

## System Architecture

```
   Host PC (OpenPnP / CNCjs)
            │  USB Serial (G-code)
            ▼
   ┌─────────────────────┐
   │      Teensy 4.1       │   Marlin / grblHAL — CAN master
   │  Custom M155 G-code    │   FlexCAN_T4 driver
   └──────────┬───────────┘
              │  MCP2551 transceiver
      ────────┴──────────  CAN bus, 500 kbps, CANH/CANL, 120Ω terminated
              │  MCP2551 transceiver
   ┌──────────┴───────────┐
   │   STM32F103 Blue Pill  │   CAN slave / local G-code engine
   └──────────┬───────────┘
              ▼
   Steppers · Feeders · Solenoids · Limit switches
```

The Teensy master parses G-code (notably a redefined `M155` command), packages structured command frames, and transmits them over CAN. The STM32 slave decodes each frame — either as a fixed motor/feeder command or, in the final architecture, as locally re-parsed G-code — and drives the connected actuators directly.

---

## Hardware

| Component | Role |
|---|---|
| Teensy 4.1 | CAN master; runs grblHAL or Marlin; generates CAN frames from G-code |
| STM32F103C8 "Blue Pill" | CAN slave; decodes frames and drives outputs; later runs a local G-code parser |
| MCP2551 CAN transceivers (×2) | Convert CAN controller logic to differential CANH/CANL |
| TB6600 (X/Y), TMC2209 standalone (Z), A4988 (E0) | Stepper drivers for axis/feeder motion |
| 28-channel feeder PCB | Feeder actuation output stage |
| Hantek DSO2D15 oscilloscope | Physical-layer CAN signal tracing |
| 24V supply | Powers feeder/motor output stage |

---

## Firmware & Software Stack

| Layer | Technology |
|---|---|
| Host G-code source | CNCjs (early phases), OpenPnP (later phases) |
| Master firmware | grblHAL → Marlin (with custom `M155` redefinition) |
| CAN driver (Teensy) | `FlexCAN_T4` |
| CAN library (STM32, evaluated) | `STM32_CAN`, `eXoCAN` |
| Slave firmware | Custom STM32F103 CAN-to-G-code bridge with local motion parser |
| Build / toolchain | PlatformIO + VS Code, Arduino IDE, `stm32flash`, Teensy Loader |
| Diagnostics | Serial monitor, multimeter, Hantek DSO2D15 oscilloscope |

---

## CAN Protocol

The protocol evolved twice: from an 8-bit feeder-only command set → a 16-bit RPM-based motor-control set → a CAN-relayed G-code model.

**Custom `M155` command (Marlin):**

```gcode
M155 L<slave_id> D<feeder/motor_index> S<speed>
```
`S` was originally an 8-bit PWM value (0–255), later upgraded to a 16-bit RPM value (0–65000) encoded across two CAN payload bytes.

**Motor-control command set:**

| Command | Purpose |
|---|---|
| `START MOTOR` | Begin rotation on the addressed channel |
| `STOP MOTOR` | Stop rotation on the addressed channel |
| `SET SPEED` | Transmit a 16-bit RPM target |
| `SET DIRECTION` | Set rotation direction |
| `EMERGENCY STOP` | Immediate halt, bypasses normal queuing |
| `PING` | Master–slave liveness check |
| `ACK` | Slave acknowledgement after execution |

**G-code supported by the STM32 CAN-to-G-code bridge (final architecture):**

| Category | Codes |
|---|---|
| Motion | `G0`, `G1`, `G28`, `G90`, `G91`, `G92` |
| Motor / state | `M17`, `M18`/`M84`, `M280` |
| Status | `M114`, `M119` |
| Sync / settings | `M400`, `M503` |

---

## Project Timeline

Six sequential phases across 30 logged working days, plus a short side engagement.

| Phase | Dates | Focus |
|---|---|---|
| 1. Setup & Study | 22–23 May | grblHAL study, CAN fundamentals, PlatformIO environment |
| 2. Feeder Control & CAN Design | 25 May – 9 Jun | Master–slave architecture, protocol design, hardware build |
| 3. Transport-Layer Debugging | 10–16 Jun | Oscilloscope-level signal tracing, isolating the FlexCAN driver fault |
| 4. grblHAL Integration | 17, 22 Jun | RPM-based protocol redesign, full end-to-end integration |
| 5. Marlin Migration | 23 Jun | Porting the validated CAN stack, physical solenoid control |
| 6. Motion Control Extension | 24–27 Jun | Stepper study, RPM calibration, CAN-to-G-code bridge |

*Side engagement (18–19 Jun): FlexoAlign Studio software study and a site demo at Kanpur Plastipack — outside the core CAN workstream.*

---

## The Core Engineering Challenge

The project's defining problem: CAN frames transmitted correctly by the Teensy master were never received by the STM32 slave — a fault that took roughly three weeks to isolate. The investigation moved methodically, layer by layer:

1. **Hardware failure** — the original Teensy 4.1 caught fire during initial bring-up, forcing a temporary return to software-only validation.
2. **Protocol verification** — confirmed CAN packet formatting and IDs matched between master and slave.
3. **Transport verification** — dedicated diagnostic firmware confirmed frames reached the STM32's receive FIFO.
4. **G-code execution path** — traced a callback-registration issue in the custom `M155` command.
5. **Electrical verification** — multimeter and register checks confirmed clean power, correct bus termination, and clean error counters.
6. **Oscilloscope-level signal tracing** — waveform capture across `CAN1_TX`, transceivers, and `CANH`/`CANL`, cross-checked against a known-working reference implementation.
7. **Driver-level root cause** — missing FlexCAN mailbox configuration, FIFO enabling, and interrupt/event servicing — confirmed when even loopback mode produced no received frames.
8. **Resolution** — rebuilt the CAN layer with correct FlexCAN initialization, resolved an unrelated grblHAL `ALARM:16` regression, and achieved full end-to-end communication.

This application → protocol → transport → electrical → physical-signal → driver progression reflects a deliberate, evidence-driven debugging methodology rather than speculative fixes.

---

## Outcomes

- A working CAN link between a Marlin-based Teensy 4.1 master and an STM32F103 slave, with the multi-week reception fault root-caused and fixed.
- A custom `M155` G-code command transmitting structured CAN packets (slave ID, feeder/motor index, 16-bit RPM).
- Hardware-validated actuator control — a solenoid physically driven end-to-end through the CAN-relayed G-code path.
- An RPM-based (not raw PWM) motor speed interface with a calibration-table-driven RPM-to-PWM conversion on the STM32 slave.
- A generalized CAN-to-G-code bridge turning the STM32 slave into a local G-code execution engine.
- A shared, centrally defined CAN protocol header used consistently across master and slave firmware.

---

## Known Limitations / Future Work

- [ ] Root-cause an intermittent runtime lock-up after several consecutive `M155` commands
- [ ] Populate the RPM-to-PWM calibration table with real measured motor data (targeting ±5–10 RPM open-loop accuracy)
- [ ] Resolve header-synchronization issues from the Phase 6 STM32 G-code engine refactor
- [ ] Add FlexCAN event servicing, CAN transmit error handling, ACK/timeout processing, and packet-length validation
- [ ] Replace blocking step generation on the STM32 with a non-blocking motion scheduler
- [ ] Expand the STM32 G-code parser's command set to match full OpenPnP output

---

## Building

Each sub-project is a self-contained [PlatformIO](https://platformio.org/) workspace.

```bash
cd Motor_Control_with_Marlin/TEENSY_4.1_CONTROL_BOARD_FIRMWARE
pio run -e teensy41 -t upload

cd ../STM32F103_Slave
pio run -e bluepill_f103c8 -t upload
```

> Update `upload_port` / `monitor_port` in each `platformio.ini` to match your local serial devices before flashing. Note: the STM32F103 cannot run USB-CDC serial debugging and CAN communication simultaneously (shared interrupts on the F1 series) — use onboard LED indication or a dedicated USB-UART adapter when debugging live CAN traffic.

---

## Skills & Tools

**Skills:** embedded C/C++ for ARM Cortex-M7 (Teensy 4.1) and Cortex-M3 (STM32F103); grblHAL/Marlin firmware architecture and plugin/user-M-code systems; CAN protocol design and low-level peripheral driver debugging (mailboxes, FIFOs, filters, interrupts); layered hardware/firmware debugging methodology; cross-firmware porting.

**Tools:** PlatformIO, VS Code, Arduino IDE, Teensy Loader, `stm32flash`, ST-Link, CNCjs, OpenPnP, Hantek DSO2D15 oscilloscope, digital multimeter.

---

## License

This project is licensed under the [MIT License](./LICENSE).

> Note: the Marlin-based sub-projects (`Feeder_Control_with_Marlin`, `Motor_Control_with_Marlin`) include Marlin firmware source, which is licensed under GPLv3 upstream. The MIT license here covers the original CAN driver, protocol, and integration code written for this project — it does not relicense the bundled Marlin codebase.
