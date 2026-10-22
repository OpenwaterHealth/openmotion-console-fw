# Open-MOTION Console Firmware Software Requirements

## 1. Document Control

- Product: Openwater Open-MOTION Console Firmware
- Repository: openmotion-console-fw
- Document type: Software Requirements Specification (baseline implementation)
- Version: 1.0
- Date: 2026-04-24
- Status: Released-behavior baseline

## 2. Purpose and Scope

This document defines software requirements for the Open-MOTION Console firmware as currently implemented in the released codebase. Requirements are derived from observed runtime behavior, command processing, and peripheral control paths already present in the firmware.

This specification is intended to:

- Provide a stable baseline for maintenance and regression control.
- Support verification and traceability for future changes.
- Capture host-interface and hardware-control behavior required for normal operation.

## 3. System Context

The firmware runs on an STM32H743-based controller and coordinates:

- Host communications over USB CDC (virtual COM packet protocol).
- Timing and trigger generation for system synchronization and laser timing.
- Sensor and monitor acquisition over I2C (through mux channels).
- TEC DAC control and TEC safety interlock behavior.
- Fan control and tachometer reporting.
- FPGA configuration/programming over I2C (MachXO2 command flow).
- Persistent runtime configuration stored in internal flash.

## 4. Operating Assumptions and Constraints

### 4.1 Platform Constraints

- The firmware shall target STM32H743 hardware with the project clock/peripheral configuration generated in current startup code.
- The firmware shall run in a no-RTOS main loop architecture.
- The host command payload maximum shall be 2048 bytes.

### 4.2 Runtime Model

- The main loop shall continuously process communications and telemetry polling.
- Peripheral initialization shall occur before entering the main loop.
- USB CDC shall be initialized before command traffic is accepted.

## 5. External Interfaces

### 5.1 Host Packet Protocol

- SI-001: The firmware shall use framed packets with start byte 0xAA and end byte 0xDD.
- SI-002: The firmware shall parse packet fields in this order: ID, packet type, command, addr, reserved, payload length, payload, CRC16.
- SI-003: The firmware shall validate CRC16 before command execution.
- SI-004: The firmware shall respond using the same transaction ID as the request.
- SI-005: On CRC failure, the firmware shall return packet type OW_BAD_CRC.
- SI-006: On unknown packet types or unsupported commands, the firmware shall return OW_UNKNOWN.

### 5.2 Transport

- SI-007: The firmware shall accept command packets over USB CDC.
- SI-008: The firmware shall support receive-to-idle style buffering for variable-length USB packets.
- SI-009: The firmware shall transmit command responses over USB CDC.

## 6. Functional Requirements

### 6.1 Startup and Initialization

- FR-001: On boot, the firmware shall initialize configured GPIO, DMA, CRC, I2C, RNG, UART, SPI, timer, and USB peripherals.
- FR-002: On boot, the firmware shall initialize debug logging and print firmware identification (version/SHA/build time).
- FR-003: On boot, the firmware shall initialize FPGA programming handle configuration for MachXO2 access.
- FR-004: On boot, the firmware shall initialize TEC DAC and set an initial output setpoint.
- FR-005: On boot, the firmware shall initialize trigger subsystem and apply default trigger configuration.
- FR-006: On boot, the firmware shall initialize I2C multiplexers, ADC monitors, TEC ADC, fan driver, and GPIO expander paths used by runtime functions.
- FR-007: On boot, the firmware shall initialize message queue and communication subsystem.
- FR-008: On boot, the firmware shall load persisted motion configuration and apply settings to runtime values/hardware mappings.

### 6.2 Global Command Set (OW_CMD)

- FR-020: The firmware shall support OW_CMD_PING and return a valid response packet.
- FR-021: The firmware shall support OW_CMD_NOP and return a valid response packet.
- FR-022: The firmware shall support OW_CMD_VERSION and return firmware version string data.
- FR-023: The firmware shall support OW_CMD_ECHO and return the input payload unchanged.
- FR-024: The firmware shall support OW_CMD_HWID and return device unique-ID words.
- FR-025: The firmware shall support OW_CMD_MESSAGES to pop queued JSON/system messages into a newline-delimited payload up to packet limits.
- FR-026: The firmware shall support OW_CMD_USR_CFG read/write operations for persisted configuration wire format and raw JSON write fallback.
- FR-027: On configuration write, the firmware shall apply updated settings immediately.
- FR-028: The firmware shall support OW_CMD_RESET by arming the reset timing path.
- FR-029: The firmware shall support OW_CMD_DFU by setting DFU entry state and arming the transition timer.

### 6.3 Controller Command Set (OW_CONTROLLER)

- FR-040: The firmware shall support I2C bus scan on selected mux index/channel and return discovered 7-bit addresses.
- FR-041: The firmware shall support setting and reading indicator LED state.
- FR-042: The firmware shall support fan PWM set command and reject malformed payloads.
- FR-043: The firmware shall support fan RPM query for supported tach channels and return RPM as 16-bit value.
- FR-044: The firmware shall support generic I2C read transaction through selected mux/channel/device/register.
- FR-045: The firmware shall support generic I2C write transaction through selected mux/channel/device/register.
- FR-046: The firmware shall support trigger configuration write from JSON and trigger configuration read as JSON.
- FR-047: The firmware shall support trigger start and trigger stop commands.
- FR-048: The firmware shall report FSYNC and LSYNC pulse counters.
- FR-049: The firmware shall support TEC DAC readback and setpoint write modes.
- FR-050: The firmware shall return telemetry samples buffer through OW_CTRL_GET_TEMPS payload.
- FR-051: The firmware shall return latest TEC status structure through OW_CTRL_TEC_STATUS payload.
- FR-052: The firmware shall support TEC ADC single-channel and all-channel read modes.
- FR-053: The firmware shall return board ID through OW_CTRL_BOARDID.
- FR-054: The firmware shall return PDU monitor frame containing raw and converted monitor values.

### 6.4 FPGA Programming Command Set (OW_FPGA_PROG)

FR-070 to FR-077 apply to Factory images only (build option `OW_FACTORY_PROGRAMMING`, CMake preset `Factory`). See FR-140 to FR-145 for every other image.

- FR-070: The firmware shall support opening FPGA configuration interface in offline mode.
- FR-071: The firmware shall support erase command with mode bitmap payload.
- FR-072: The firmware shall support CFG/UFM address reset, single-page write, and single-page read.
- FR-073: The firmware shall support Feature Row write/read operations.
- FR-074: The firmware shall support Set Done, Refresh, and Close operations.
- FR-075: The firmware shall support multi-page CFG and UFM write payloads (N pages x page size).
- FR-076: The firmware shall support FPGA status-register read and return 4-byte status payload.
- FR-077: The firmware shall reject malformed FPGA programming payload lengths with parse/data errors.

### 6.5 Telemetry and Safety

- FR-090: The firmware shall poll telemetry from the main loop at periodic intervals or timer tick request.
- FR-091: The firmware shall acquire temperature sensors and TEC ADC channels during telemetry polling.
- FR-092: The firmware shall store telemetry samples in a ring buffer and drop oldest samples when full.
- FR-093: The firmware shall maintain a latest TEC status snapshot for host queries.
- FR-094: If TEC trip threshold is configured and exceeded, the firmware shall assert safety disconnect, mark TEC status false, and queue a system error JSON message.
- FR-095: After sustained safe samples, the firmware shall clear TEC trip safety interlock.

### 6.6 Trigger Control and Interlocks

- FR-110: The firmware shall support trigger configuration with range checks (frequency and pulse-width validity).
- FR-111: Trigger start shall fail when USB trigger interlock is active.
- FR-112: Trigger stop shall disable sync outputs and TA trigger output.
- FR-113: USB disconnect events shall force trigger stop by interlock.
- FR-114: Safety disconnect shall force trigger stop; safety clear shall release the interlock.

### 6.7 Persistent Configuration

- FR-130: The firmware shall persist motion configuration in internal flash with magic/version/sequence/CRC and JSON blob.
- FR-131: On invalid persisted configuration (magic/version/CRC/termination) or an unmigratable record, firmware shall run on defaults in RAM, report it and refuse the trigger (CFW-03); the stored record is left for inspection.
- FR-132: The firmware shall expose configuration as a wire format header plus JSON data for host read.
- FR-133: The firmware shall accept both full wire payload and raw JSON payload for host write.
- FR-134: The firmware shall increment sequence and recompute CRC on each committed configuration write.
- FR-135: Operational configuration keys shall be scaled, range-checked and written to their mapped FPGA registers during apply; safety and unknown keys shall be ignored and logged (FR-150 to FR-155).
- FR-136: The TEC trip voltage threshold shall be derived from the compile-time SAFETY_TEC_TRIP_C (FR-150); Factory images may override it from the record within hard bounds.

### 6.8 FPGA Programming Lock-out and Configuration Attestation (PTR-2026-1-5)

- FR-140: A Release or Debug image shall contain no FPGA or UFM programming code; the release pipeline shall fail if the image contains any programming symbol.
- FR-141: A Release or Debug image shall answer every OW_FPGA_PROG command with OW_ERROR and the data byte OW_CODE_NOT_SUPPORTED.
- FR-142: After initialisation, and on OW_CMD_FPGA_ATTEST request outside a scan, the firmware shall read the configuration flash of each of the two safety MachXO2 devices (EE, OPT) in transparent mode, compute its SHA-256 and compare it with the digest compiled into the image from the released safety bitstream artefact; the seed and TA devices shall be reported as not checked.
- FR-146: I2C1 (mux 0 and the FPGA configuration ports) shall run in fast mode (at most 400 kHz); the attestation of both safety devices shall complete within 10 s of its start.
- FR-143: For each safety FPGA the firmware shall read the per-unit calibration record from UFM and validate its magic, layout version, device, checksum and every limit against the hard bounds.
- FR-144: A digest mismatch, read failure or invalid record shall set TRIG_INTERLOCK_FPGA_ATTEST, refuse trigger start (and configuration after a failed run), stop a running trigger, queue a fault message and be reported in the attestation and boot-information replies; the interlock shall persist until a run passes.
- FR-145: The attestation shall not block command handling: it shall run from the main loop within a bounded bus-time budget per iteration.

### 6.9 Firmware-owned Safety Parameters

- FR-150: The TEC trip threshold, the EE/OPT limit registers and the laser-driver clamps shall be constants of the firmware image, in force from reset; the TEC trip threshold shall not depend on the configuration record.
- FR-151: The firmware shall write and read back the firmware-owned registers at boot, after every configuration write and after every completed attestation; a failure shall set TRIG_INTERLOCK_LIMITS.
- FR-152: A valid per-unit UFM record shall replace the compile-time EE/OPT limits for that device and may lower the operational ceilings; a blank UFM shall be reported and shall not be an interlock.
- FR-153: The configuration record (schema v3) shall hold only allow-listed operational keys and the opaque calibration block; a host write naming a safety key, an unknown key, or an out-of-range value shall be refused with OW_CODE_CFG_SAFETY_KEY, OW_CODE_CFG_UNKNOWN_KEY or OW_CODE_CFG_RANGE and nothing shall be stored.
- FR-154: A v1/v2 record shall be migrated at load by keeping in-range operational and opaque keys and dropping the others, or rejected as a whole; it shall never be half-applied.
- FR-155: OW_CTRL_I2C_WR shall refuse writes to firmware-owned registers and accept writes to operational registers only within their ranges.
- FR-156: Factory images may accept safety keys as overrides within the hard bounds, shall announce themselves at boot, in the version string and in boot-info, and shall never be built for a release tag.

## 7. Error Handling Requirements

- ER-001: Invalid command payload lengths shall result in error or parse-failure packet responses.
- ER-002: I2C transaction failures in command handlers shall result in OW_ERROR response behavior for the affected command.
- ER-003: Unsupported command modes (for example invalid reserved mode values) shall return OW_UNKNOWN or OW_ERROR per command implementation.
- ER-004: Flash/config write failures shall result in operation failure status and no false success response.

## 8. Performance and Timing Requirements

- PR-001: Telemetry polling shall support nominal 25 ms polling cadence fallback when no timer tick is pending.
- PR-002: TIM4 telemetry tick configuration shall provide approximately 250 ms periodic interrupt-driven poll request.
- PR-003: Command processing shall be non-RTOS and execute in the foreground loop with bounded packet size.

## 9. Verification Approach

The requirements in this document should be verified using:

- Static verification against implemented command handlers and modules.
- Host protocol tests for packet framing, CRC handling, and command responses.
- Hardware-in-loop tests for trigger timing, safety interlocks, telemetry acquisition, fan control, and FPGA programming operations.
- Persistence tests for configuration read/write, CRC validation, default recovery, and reboot retention.

## 10. Out-of-Scope for This Baseline

- New protocol features not currently implemented.
- New hardware support beyond currently initialized peripherals and mapped devices.
- UI, application-layer host software requirements.

## 11. Traceability Notes

This document intentionally reflects currently implemented behavior and command semantics so it can be used as a released-baseline software requirements set. Future changes should revise requirement IDs and include validation evidence.
