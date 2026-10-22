Here’s a **concise, matrix-ready set of test cases mapped to each SR**. Each test is atomic, verifiable, and suitable for HIL or bench testing.

Format:
- **TC-ID**
- **Maps to SR**
- **Description**
- **Method** (Test / HIL / Inspection)
- **Pass Criteria**

---

### System

- TC-SYS-001  
  Maps to: SR-001  
  Description: Verify firmware runs on STM32H743 target  
  Method: Inspection  
  Pass: Firmware builds and executes on target hardware  

- TC-SYS-002  
  Maps to: SR-002  
  Description: Verify no RTOS usage  
  Method: Inspection  
  Pass: No RTOS scheduler or threads present  

- TC-SYS-003  
  Maps to: SR-003  
  Description: Verify all peripherals initialized before main loop  
  Method: Inspection / HIL  
  Pass: Peripherals respond before command handling begins  

- TC-SYS-004  
  Maps to: SR-004  
  Description: Verify continuous loop execution  
  Method: HIL  
  Pass: Commands and telemetry processed continuously  

---

### Communication

- TC-COM-001  
  Maps to: SR-COM-001  
  Description: Send valid command over USB CDC  
  Method: Test  
  Pass: Command received and processed  

- TC-COM-002  
  Maps to: SR-COM-002  
  Description: Verify response transmission  
  Method: Test  
  Pass: Response packet received by host  

- TC-COM-003  
  Maps to: SR-COM-003  
  Description: Validate packet framing  
  Method: Test  
  Pass: Only packets with 0xAA/0xDD accepted  

- TC-COM-004  
  Maps to: SR-COM-004  
  Description: Send valid CRC packet  
  Method: Test  
  Pass: Packet accepted  

- TC-COM-005  
  Maps to: SR-COM-005  
  Description: Send invalid CRC packet  
  Method: Test  
  Pass: OW_BAD_CRC returned  

- TC-COM-006  
  Maps to: SR-COM-006  
  Description: Verify transaction ID echo  
  Method: Test  
  Pass: Response ID matches request  

- TC-COM-007  
  Maps to: SR-COM-007  
  Description: Send max payload (2048 bytes)  
  Method: Test  
  Pass: Packet processed successfully  

- TC-COM-008  
  Maps to: SR-COM-008  
  Description: Send variable-length packets  
  Method: Test  
  Pass: All lengths correctly received  

---

### Command Processing

- TC-CMD-001  
  Maps to: SR-CMD-001  
  Description: Send multiple command types  
  Method: Test  
  Pass: Correct handler invoked  

- TC-CMD-002  
  Maps to: SR-CMD-002  
  Description: Execute ping/version/echo  
  Method: Test  
  Pass: Correct responses returned  

- TC-CMD-003  
  Maps to: SR-CMD-003  
  Description: Write and read configuration  
  Method: Test  
  Pass: Data matches  

- TC-CMD-004  
  Maps to: SR-CMD-004  
  Description: Issue reset/DFU command  
  Method: HIL  
  Pass: Device transitions correctly  

- TC-CMD-005  
  Maps to: SR-CMD-005  
  Description: Send unknown command  
  Method: Test  
  Pass: OW_UNKNOWN returned  

---

### Hardware

- TC-HW-001  
  Maps to: SR-HW-001  
  Description: Select mux channel and access device  
  Method: HIL  
  Pass: Device responds  

- TC-HW-002  
  Maps to: SR-HW-002  
  Description: Perform I2C read  
  Method: HIL  
  Pass: Correct data returned  

- TC-HW-003  
  Maps to: SR-HW-003  
  Description: Perform I2C write  
  Method: HIL  
  Pass: Device state updated  

- TC-HW-004  
  Maps to: SR-HW-004  
  Description: Run I2C scan  
  Method: HIL  
  Pass: Valid addresses returned  

- TC-HW-005  
  Maps to: SR-HW-005  
  Description: Toggle LED  
  Method: HIL  
  Pass: LED state changes  

- TC-HW-006  
  Maps to: SR-HW-006  
  Description: Set fan PWM  
  Method: HIL  
  Pass: Fan speed changes  

- TC-HW-007  
  Maps to: SR-HW-007  
  Description: Read fan RPM  
  Method: HIL  
  Pass: Valid RPM returned  

- TC-HW-008  
  Maps to: SR-HW-008  
  Description: Set TEC DAC  
  Method: HIL  
  Pass: Output voltage changes  

- TC-HW-009  
  Maps to: SR-HW-009  
  Description: Read TEC ADC  
  Method: HIL  
  Pass: Valid readings returned  

- TC-HW-010  
  Maps to: SR-HW-010  
  Description: Communicate with FPGA  
  Method: HIL  
  Pass: FPGA responds  

---

### FPGA

- TC-FPGA-001  
  Maps to: SR-FPGA-001  
  Description: Open/close interface  
  Method: HIL  
  Pass: No errors  

- TC-FPGA-002  
  Maps to: SR-FPGA-002  
  Description: Execute erase  
  Method: HIL  
  Pass: Memory cleared  

- TC-FPGA-003  
  Maps to: SR-FPGA-003  
  Description: Read/write page  
  Method: HIL  
  Pass: Data matches  

- TC-FPGA-004  
  Maps to: SR-FPGA-004  
  Description: Multi-page write  
  Method: HIL  
  Pass: All pages written  

- TC-FPGA-005  
  Maps to: SR-FPGA-005  
  Description: Read status register  
  Method: HIL  
  Pass: Valid 4-byte response  

- TC-FPGA-006  
  Maps to: SR-FPGA-006  
  Description: Send invalid payload  
  Method: Test  
  Pass: Error returned  

- TC-FPGA-007
  Maps to: SR-ATTEST-001
  Description: OW_FPGA_PROG on a Release/Debug image
  Method: HIL (tests/test_safety_state_hil.py)
  Pass: OW_ERROR + OW_CODE_NOT_SUPPORTED for every opcode; Release ELF has no programming symbol (CI nm check)

- TC-FPGA-008
  Maps to: SR-ATTEST-002
  Description: Boot attestation on a correctly programmed unit
  Method: HIL
  Pass: EE and OPT match with UFM valid, seed and TA reported not checked, interlock 0x40 clears, run complete within 10 s (FR-146), readback time recorded

- TC-FPGA-009
  Maps to: SR-ATTEST-003
  Description: Modified bitstream on one device (bench-only reprogramming)
  Method: HIL (OPENMOTION_HIL_REPROGRAM=1)
  Pass: digest mismatch reported, 0x40 set, trigger start refused with 0x40, boot-info shows state FAIL

- TC-FPGA-010
  Maps to: SR-ATTEST-002
  Description: Digest pipeline and UFM record validation
  Method: Test (host_tests: test_fpga_digest, test_safety_ufm_record, test_sha256)
  Pass: NIST vectors, page-stream digest equals image digest, flipped bit / short read / swapped page detected; record good / FPGA reference record byte-identical / bad checksum / outside envelope (each field, LL > UL) / blank / old layout / layout / reserved; loads only from a valid record; setpoint ceilings; envelope LSBs equal the register map

- TC-FPGA-011
  Maps to: SR-ATTEST-002, SR-SAFEP-001
  Description: Unit without a valid safety record (blank UFM) on a Release image
  Method: HIL (test_missing_or_invalid_record_keeps_interlock)
  Pass: digests match, device status 0x08 (blank) or 0x04 (invalid), run FAIL, interlock 0x40 set, boot-info state FAIL, trigger start refused with 0x40

- TC-SAFEP-010
  Maps to: SR-SAFEP-001, SR-SAFEP-002
  Description: Per-unit ceilings on the operating point
  Method: Test (host_tests: test_motion_cfg_keys)
  Pass: a ceiling below the envelope refuses a larger TA_CURRENT_DRV / TA_PULSE_WIDTH on the config and pass-through paths; a ceiling above the envelope leaves the envelope; safety keys take no ceiling; Factory TEC_TRIP override above 40 C refused

---

### Telemetry

- TC-TLM-001  
  Maps to: SR-TLM-001  
  Description: Verify periodic polling  
  Method: HIL  
  Pass: Data updates periodically  

- TC-TLM-002  
  Maps to: SR-TLM-002  
  Description: Read temperature sensors  
  Method: HIL  
  Pass: Valid values  

- TC-TLM-003  
  Maps to: SR-TLM-003  
  Description: Read TEC ADC channels  
  Method: HIL  
  Pass: Valid values  

- TC-TLM-004  
  Maps to: SR-TLM-004  
  Description: Fill telemetry buffer  
  Method: Test  
  Pass: Buffer contains samples  

- TC-TLM-005  
  Maps to: SR-TLM-005  
  Description: Overflow buffer  
  Method: Test  
  Pass: Oldest data dropped  

- TC-TLM-006  
  Maps to: SR-TLM-006  
  Description: Request telemetry  
  Method: Test  
  Pass: Data returned  

- TC-TLM-007  
  Maps to: SR-TLM-007  
  Description: Query latest snapshot  
  Method: Test  
  Pass: Latest values returned  

---

### Safety

- TC-SAFE-001  
  Maps to: SR-SAFE-001  
  Description: Exceed TEC threshold  
  Method: HIL  
  Pass: Trip detected  

- TC-SAFE-002  
  Maps to: SR-SAFE-002  
  Description: Verify TEC shutdown  
  Method: HIL  
  Pass: Output disabled  

- TC-SAFE-003  
  Maps to: SR-SAFE-003  
  Description: Trigger during fault  
  Method: HIL  
  Pass: Trigger blocked  

- TC-SAFE-004  
  Maps to: SR-SAFE-004  
  Description: Disconnect USB  
  Method: HIL  
  Pass: Trigger stops  

- TC-SAFE-005  
  Maps to: SR-SAFE-005  
  Description: Check error message  
  Method: Test  
  Pass: JSON message queued  

- TC-SAFE-006  
  Maps to: SR-SAFE-006  
  Description: Restore safe condition  
  Method: HIL  
  Pass: Interlock cleared  

---

### Trigger

- TC-TRIG-001  
  Maps to: SR-TRIG-001  
  Description: Configure trigger  
  Method: Test  
  Pass: Config applied  

- TC-TRIG-002  
  Maps to: SR-TRIG-002  
  Description: Send invalid config  
  Method: Test  
  Pass: Rejected  

- TC-TRIG-003  
  Maps to: SR-TRIG-003  
  Description: Start trigger  
  Method: HIL  
  Pass: Output active  

- TC-TRIG-004  
  Maps to: SR-TRIG-004  
  Description: Stop trigger  
  Method: HIL  
  Pass: Output inactive  

- TC-TRIG-005  
  Maps to: SR-TRIG-005  
  Description: Read counters  
  Method: Test  
  Pass: Values increment  

- TC-TRIG-006  
  Maps to: SR-TRIG-006  
  Description: Start with interlock  
  Method: HIL  
  Pass: Start denied  

---

### Configuration

- TC-CFG-001  
  Maps to: SR-CFG-001  
  Description: Save configuration  
  Method: Test  
  Pass: Stored in flash  

- TC-CFG-002  
  Maps to: SR-CFG-002  
  Description: Corrupt config  
  Method: Test  
  Pass: Detected invalid  

- TC-CFG-003  
  Maps to: SR-CFG-003  
  Description: Boot with invalid config  
  Method: HIL  
  Pass: Defaults restored  

- TC-CFG-004  
  Maps to: SR-CFG-004  
  Description: Read config  
  Method: Test  
  Pass: Data returned  

- TC-CFG-005  
  Maps to: SR-CFG-005  
  Description: Write config  
  Method: Test  
  Pass: Stored correctly  

- TC-CFG-006  
  Maps to: SR-CFG-006  
  Description: Apply config immediately  
  Method: HIL  
  Pass: Behavior changes  

- TC-CFG-007  
  Maps to: SR-CFG-007  
  Description: Verify sequence increment  
  Method: Test  
  Pass: Sequence increases  

- TC-CFG-008  
  Maps to: SR-CFG-008  
  Description: Verify HW mapping  
  Method: HIL  
  Pass: Registers updated  
- TC-CFG-009
  Maps to: SR-SAFEP-002
  Description: Config write with a safety key, unknown key, out-of-range value
  Method: Test (host_tests/test_motion_cfg_keys.c) + HIL
  Pass: refused with OW_CODE_CFG_SAFETY_KEY / CFG_UNKNOWN_KEY / CFG_RANGE; stored record and seq unchanged

- TC-CFG-010
  Maps to: SR-SAFEP-002
  Description: Record migration v2 to v3
  Method: Test (host_tests/test_motion_cfg_keys.c)
  Pass: operational and opaque keys kept verbatim, safety and unknown keys dropped, out-of-range values dropped, never half-applied

- TC-CFG-011
  Maps to: SR-SAFEP-001
  Description: TEC trip in force after record corruption
  Method: HIL
  Pass: after a refused write and an empty record the trip evaluates from the compile-time threshold; no defaults interlock for an empty record

- TC-CFG-012
  Maps to: SR-SAFEP-002
  Description: Pass-through write to a firmware-owned register
  Method: Test (host_tests) + HIL
  Pass: refused "firmware-owned safety register"; operational register accepted only within range

---

### Errors

- TC-ERR-001  
  Maps to: SR-ERR-001  
  Description: Send bad payload length  
  Method: Test  
  Pass: Error returned  

- TC-ERR-002  
  Maps to: SR-ERR-002  
  Description: Invalid command format  
  Method: Test  
  Pass: Error response  

- TC-ERR-003  
  Maps to: SR-ERR-003  
  Description: Force I2C failure  
  Method: HIL  
  Pass: Error reported  

- TC-ERR-004  
  Maps to: SR-ERR-004  
  Description: Simulate flash failure  
  Method: Test  
  Pass: Failure reported  

- TC-ERR-005  
  Maps to: SR-ERR-005  
  Description: Force operation failure  
  Method: Test  
  Pass: No success response  

---

### Performance

- TC-PERF-001  
  Maps to: SR-PERF-001  
  Description: Measure 25 ms polling  
  Method: HIL  
  Pass: Within tolerance  

- TC-PERF-002  
  Maps to: SR-PERF-002  
  Description: Measure timer polling  
  Method: HIL  
  Pass: ~250 ms interval  

- TC-PERF-003  
  Maps to: SR-PERF-003  
  Description: Stress command load  
  Method: Test  
  Pass: No missed responses  

---

### Initialization

- TC-INIT-001  
  Maps to: SR-INIT-001  
  Description: Power-on initialization  
  Method: HIL  
  Pass: All peripherals active  

- TC-INIT-002  
  Maps to: SR-INIT-002  
  Description: Verify USB ready before commands  
  Method: Test  
  Pass: No early command accepted  

- TC-INIT-003  
  Maps to: SR-INIT-003  
  Description: Check safe defaults  
  Method: HIL  
  Pass: No unsafe outputs  

- TC-INIT-004  
  Maps to: SR-INIT-004  
  Description: Load persisted config  
  Method: HIL  
  Pass: Config applied  

---

If you want next step, I can:
- Turn this into a **full RTM spreadsheet (CSV/Excel import)**
- Add **test procedures (step-by-step)**
- Add **pass/fail logs + automation hooks (Python test harness)**