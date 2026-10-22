Here are **structured test procedures** for representative and critical requirements. These are written so they can be directly used in a test system (manual or automated HIL). You can replicate the pattern for full coverage.

---

## Communication Test Procedures

### TP-COM-001: Valid Packet Processing
**Maps to:** SR-COM-001, SR-COM-004  
**Setup:**
- Connect device via USB CDC
- Open serial interface

**Steps:**
1. Construct valid packet with correct framing and CRC.
2. Send packet to device.
3. Capture response.

**Expected Result:**
- Device responds with valid packet.
- CRC is accepted and command executed.

---

### TP-COM-002: CRC Error Handling
**Maps to:** SR-COM-005  
**Steps:**
1. Construct packet with invalid CRC.
2. Send to device.
3. Capture response.

**Expected Result:**
- Device returns `OW_BAD_CRC`.
- No command execution occurs.

---

### TP-COM-003: Max Payload Handling
**Maps to:** SR-COM-007  
**Steps:**
1. Create packet with 2048-byte payload.
2. Send to device.
3. Monitor response.

**Expected Result:**
- Packet processed successfully.
- No truncation or crash.

---

## Command Processing Procedures

### TP-CMD-001: Ping Command
**Maps to:** SR-CMD-002  
**Steps:**
1. Send OW_CMD_PING packet.
2. Capture response.

**Expected Result:**
- Valid response returned with matching transaction ID.

---

### TP-CMD-002: Unknown Command Handling
**Maps to:** SR-CMD-005  
**Steps:**
1. Send packet with undefined command ID.
2. Capture response.

**Expected Result:**
- Device returns `OW_UNKNOWN`.

---

### TP-CMD-003: Configuration Write/Read
**Maps to:** SR-CMD-003  
**Steps:**
1. Send configuration write packet (JSON or wire format).
2. Send configuration read request.
3. Compare results.

**Expected Result:**
- Returned configuration matches written values.
- Changes applied immediately.

---

## Hardware Interface Procedures

### TP-HW-001: I2C Read/Write
**Maps to:** SR-HW-002, SR-HW-003  
**Setup:**
- Connect known I2C device

**Steps:**
1. Write value to register via command.
2. Read back register.
3. Compare values.

**Expected Result:**
- Read value matches written value.

---

### TP-HW-002: Fan Control and RPM
**Maps to:** SR-HW-006, SR-HW-007  
**Steps:**
1. Set fan PWM to defined value.
2. Wait for stabilization.
3. Query RPM.

**Expected Result:**
- RPM corresponds to PWM setting.

---

### TP-HW-003: TEC DAC/ADC
**Maps to:** SR-HW-008, SR-HW-009  
**Steps:**
1. Set TEC DAC output.
2. Read TEC ADC channel.
3. Compare expected voltage relationship.

**Expected Result:**
- ADC reflects DAC output within tolerance.

---

## FPGA Procedures

### TP-FPGA-001: Page Write/Read
**Maps to:** SR-FPGA-003  
**Steps:**
1. Write data to FPGA page.
2. Read same page.
3. Compare data.

**Expected Result:**
- Data matches exactly.

---

### TP-FPGA-002: Invalid Payload Handling
**Maps to:** SR-FPGA-006  
**Steps:**
1. Send malformed FPGA command.
2. Capture response.

**Expected Result:**
- Error response returned.

---

### TP-FPGA-003: Attestation Positive Path
**Maps to:** SR-ATTEST-002
**Steps:**
1. Power the console with a Release or Debug image; capture the debug UART.
2. Poll OW_CMD_FPGA_ATTEST until state is PASS or FAIL.
3. Read OW_CMD_BOOT_INFO.

**Expected Result:**
- Precondition: both safety devices provisioned with a safety record (fpga-attestation.md §6.4).
- State PASS; EE and OPT: no fault bit, 3198 pages, digest equals expected, UFM valid (status bit 0x10); seed and TA: status 0x40 (not checked).
- Both readbacks together complete within 10 s (about 3.2 s each at 400 kHz).
- Boot-info interlock byte bit 0x40 clear; trigger start no longer refused for 0x40.

---

### TP-FPGA-005: Missing Safety Record
**Maps to:** SR-ATTEST-002, SR-SAFEP-001
**Steps:**
1. On a unit whose EE or OPT UFM is blank (never provisioned), power the console with a Release image.
2. Poll OW_CMD_FPGA_ATTEST until state is PASS or FAIL; read OW_CMD_BOOT_INFO; send OW_CTRL_START_TRIG.

**Expected Result:**
- EE and OPT digests equal expected; the unprovisioned device(s) report status bit 0x08 and ufm_status 0 (blank).
- State FAIL, interlocked 1; boot-info attestation state 3, interlock bit 0x40 set.
- OW_CTRL_START_TRIG refused with 0x40 in the status byte.
- Debug UART: "Safety record: EE none, OPT none; ... the laser stays interlocked".

---

### TP-FPGA-004: Programming Refused
**Maps to:** SR-ATTEST-001
**Steps:**
1. Send OW_FPGA_PROG opcodes 0x3F and 0x30 for channels 0..3.

**Expected Result:**
- Every reply is OW_ERROR with the single data byte 0xFB.

---

### TP-FPGA-005: Modified Bitstream (bench unit only)
**Maps to:** SR-ATTEST-003
**Steps:**
1. Factory image: program one device with a bitstream differing from the release.
2. Flash the Release image; wait for the attestation to finish.
3. Send OW_CTRL_START_TRIG.

**Expected Result:**
- State FAIL with the device's digest bit set; start refused with 0x40 in the status byte; fault message queued.
- Recovery per docs/fpga-attestation.md section 6.2 step 4 restores PASS.

---

## Telemetry Procedures

### TP-TLM-001: Periodic Telemetry
**Maps to:** SR-TLM-001  
**Steps:**
1. Enable telemetry polling.
2. Monitor values over time.

**Expected Result:**
- Data updates at expected interval (~25 ms or timer-driven).

---

### TP-TLM-002: Buffer Overflow
**Maps to:** SR-TLM-005  
**Steps:**
1. Continuously poll telemetry until buffer fills.
2. Continue polling beyond capacity.
3. Inspect buffer.

**Expected Result:**
- Oldest samples overwritten.
- No crash or corruption.

---

## Safety Procedures

### TP-SAFE-001: TEC Trip Protection
**Maps to:** SR-SAFE-001, SR-SAFE-002  
**Steps:**
1. Configure TEC trip threshold.
2. Force temperature/voltage above threshold.
3. Observe system behavior.

**Expected Result:**
- TEC disabled.
- Safety flag set.
- Error message generated.

---

### TP-SAFE-002: USB Disconnect Interlock
**Maps to:** SR-SAFE-004  
**Steps:**
1. Start trigger.
2. Physically disconnect USB.
3. Observe outputs.

**Expected Result:**
- Trigger stops immediately.

---

## Trigger Procedures

### TP-TRIG-001: Trigger Start/Stop
**Maps to:** SR-TRIG-003, SR-TRIG-004  
**Steps:**
1. Configure valid trigger.
2. Send start command.
3. Verify output signal.
4. Send stop command.

**Expected Result:**
- Output active on start.
- Output stops on command.

---

### TP-TRIG-002: Interlock Enforcement
**Maps to:** SR-TRIG-006  
**Steps:**
1. Activate safety or USB interlock.
2. Attempt trigger start.

**Expected Result:**
- Start command rejected.

---

## Configuration Procedures

### TP-CFG-001: Persistence Across Reboot
**Maps to:** SR-CFG-001  
**Steps:**
1. Write configuration.
2. Reboot device.
3. Read configuration.

**Expected Result:**
- Configuration retained.

---

### TP-CFG-002: Corrupt Configuration Recovery
**Maps to:** SR-CFG-003  
**Steps:**
1. Corrupt flash configuration manually.
2. Reboot device.

**Expected Result:**
- Defaults restored.
- New valid config written.

---

### TP-CFG-003: Safety Key Rejected
**Maps to:** SR-SAFEP-002
**Steps:**
1. Read the stored config (seq).
2. Write `{"TEC_TRIP": 90}`, `{"EE_DRIVE_CL": 9999}`, `{"TA_CURRENT_DRV": 9000}`, `{"bogus": 1}`.
3. Read the stored config again.

**Expected Result:**
- OW_ERROR with 0xFA, 0xFA, 0xF8, 0xF9 respectively; seq and text unchanged.

---

### TP-CFG-004: TEC Trip Independent of the Record
**Maps to:** SR-SAFEP-001
**Steps:**
1. Write a non-strict config (`{"TA_CURRENT_DRV": NaN}`), then `{}`.
2. Read OW_CTRL_TEC_STATUS and OW_CMD_BOOT_INFO.

**Expected Result:**
- First write refused, second accepted; TEC status evaluating (trip armed from the compile-time threshold); no defaults interlock.

---

## Error Handling Procedures

### TP-ERR-001: Invalid Payload Length
**Maps to:** SR-ERR-001  
**Steps:**
1. Send packet with incorrect payload length field.
2. Capture response.

**Expected Result:**
- Error returned.
- No execution.

---

### TP-ERR-002: I2C Failure Handling
**Maps to:** SR-ERR-003  
**Steps:**
1. Disconnect I2C device.
2. Issue I2C command.

**Expected Result:**
- Error response returned.

---

## Performance Procedures

### TP-PERF-001: Telemetry Timing
**Maps to:** SR-PERF-001  
**Steps:**
1. Capture timestamps of telemetry updates.
2. Calculate interval.

**Expected Result:**
- ~25 ms ± tolerance.

---

### TP-PERF-002: Command Load Stress
**Maps to:** SR-PERF-003  
**Steps:**
1. Send continuous command stream.
2. Monitor responses.

**Expected Result:**
- No dropped or delayed responses beyond acceptable limits.

---

## Initialization Procedures

### TP-INIT-001: Power-On Initialization
**Maps to:** SR-INIT-001  
**Steps:**
1. Power cycle device.
2. Observe initialization logs and behavior.

**Expected Result:**
- All peripherals initialized.
- No errors.

---

### TP-INIT-002: Safe Defaults
**Maps to:** SR-INIT-003  
**Steps:**
1. Boot device without commands.
2. Measure outputs (TEC, trigger, etc.).

**Expected Result:**
- System in safe, inactive state.

---

## If you want next step

I can:
- Turn this into a **fully formatted test procedure document (ready for QA sign-off)**  
- Generate a **Python-based automated test harness (USB CDC driver + packet builder)**  
- Or build a **complete RTM (HL ↔ SR ↔ FR ↔ TC ↔ TP)**