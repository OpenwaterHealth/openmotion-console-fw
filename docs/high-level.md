### Open-MOTION Console Firmware  
### High-Level Software Requirements (HL-SRS)

**Version:** 1.0  
**Date:** 2026-04-24  
**Scope:** High-level requirements derived from baseline implementation

---

### 1. System Overview

The firmware shall operate as an embedded control system on an STM32H743 platform that manages communication between a host system and hardware subsystems including sensors, TEC control, FPGA configuration, triggers, and system telemetry.

The firmware shall provide deterministic control, monitoring, and safety enforcement without reliance on an RTOS.

---

### 2. Architectural Requirements

- HL-AR-001: The firmware shall implement a single-threaded, main-loop execution model.
- HL-AR-002: The firmware shall initialize all required peripherals and subsystems prior to entering runtime operation.
- HL-AR-003: The firmware shall modularize functionality into communication, control, telemetry, safety, and persistence subsystems.
- HL-AR-004: The firmware shall maintain a clear separation between host interface handling and hardware control logic.

---

### 3. Communication Requirements

- HL-COM-001: The firmware shall provide a bidirectional command/response interface over USB CDC.
- HL-COM-002: The firmware shall implement a framed packet protocol with integrity validation.
- HL-COM-003: The firmware shall process incoming commands and return deterministic responses.
- HL-COM-004: The firmware shall handle malformed, unsupported, or corrupted packets with explicit error responses.
- HL-COM-005: The firmware shall support variable-length payloads up to defined limits.

---

### 4. Command and Control Requirements

- HL-CTRL-001: The firmware shall expose a structured command set for system, controller, and FPGA operations.
- HL-CTRL-002: The firmware shall support device introspection commands (e.g., version, ID, status).
- HL-CTRL-003: The firmware shall support runtime configuration updates from the host.
- HL-CTRL-004: The firmware shall execute hardware control commands for I2C devices, fans, LEDs, and TEC subsystems.
- HL-CTRL-005: The firmware shall support trigger configuration, activation, and deactivation.
- HL-CTRL-006: The firmware shall support FPGA status monitoring and configuration attestation; programming workflows exist in Factory images only.

---

### 5. Hardware Interface Requirements

- HL-HW-001: The firmware shall interface with peripherals including GPIO, I2C, SPI, UART, DMA, timers, and USB.
- HL-HW-002: The firmware shall support multiplexed I2C communication for multiple downstream devices.
- HL-HW-003: The firmware shall control TEC DAC outputs and read TEC ADC inputs.
- HL-HW-004: The firmware shall control fan speed via PWM and monitor tachometer feedback.
- HL-HW-005: The firmware shall configure and communicate with an FPGA over I2C.
- HL-HW-006: The firmware shall manage trigger outputs and synchronization signals.

---

### 6. Telemetry and Monitoring Requirements

- HL-TLM-001: The firmware shall periodically acquire system telemetry including temperatures, TEC data, and monitor channels.
- HL-TLM-002: The firmware shall maintain a buffer of recent telemetry samples.
- HL-TLM-003: The firmware shall provide telemetry data to the host upon request.
- HL-TLM-004: The firmware shall maintain a latest-state snapshot for critical subsystems.

---

### 7. Safety and Interlock Requirements

- HL-SAFE-001: The firmware shall enforce safety interlocks on trigger and TEC subsystems.
- HL-SAFE-002: The firmware shall detect fault conditions (e.g., TEC trip thresholds) and take protective action.
- HL-SAFE-003: The firmware shall disable triggering on safety violations or communication loss.
- HL-SAFE-004: The firmware shall generate and queue system error notifications for fault events.
- HL-SAFE-005: The firmware shall support recovery from fault conditions when safe operating criteria are restored.
- HL-SAFE-006: The firmware shall verify at boot and on request that the configuration of every laser-driver and safety FPGA is the released design, and shall not allow the laser to be armed until it has; a shipped image shall contain no means of reprogramming those devices.
- HL-SAFE-007: The laser-safety parameters (thermal trip, safety-monitor limits, driver clamps) shall be owned by the firmware image, in force from reset and not writable by the host; the host shall only set operational parameters within firmware-validated ranges.

---

### 8. Configuration and Persistence Requirements

- HL-CFG-001: The firmware shall store configuration data persistently in internal flash.
- HL-CFG-002: The firmware shall validate stored configuration using integrity checks.
- HL-CFG-003: The firmware shall restore default configuration when stored data is invalid.
- HL-CFG-004: The firmware shall allow host read/write access to configuration data.
- HL-CFG-005: The firmware shall apply configuration changes immediately to runtime behavior and hardware.
- HL-CFG-006: The firmware shall map configuration parameters to hardware register settings where applicable.

---

### 9. Startup and Initialization Requirements

- HL-INIT-001: The firmware shall initialize all hardware peripherals and software subsystems on boot.
- HL-INIT-002: The firmware shall load and apply persisted configuration during startup.
- HL-INIT-003: The firmware shall initialize communication interfaces before accepting commands.
- HL-INIT-004: The firmware shall establish safe default states for all controlled hardware.

---

### 10. Error Handling Requirements

- HL-ERR-001: The firmware shall detect and report invalid inputs, communication errors, and hardware failures.
- HL-ERR-002: The firmware shall prevent execution of invalid or unsafe operations.
- HL-ERR-003: The firmware shall provide explicit error responses to the host.
- HL-ERR-004: The firmware shall ensure no false-positive success responses on failed operations.

---

### 11. Performance Requirements

- HL-PERF-001: The firmware shall process commands within the constraints of a non-RTOS main loop.
- HL-PERF-002: The firmware shall support periodic telemetry polling at defined intervals.
- HL-PERF-003: The firmware shall maintain responsiveness to host commands during telemetry and background tasks.

---

### 12. Maintainability and Traceability Requirements

- HL-MNT-001: The firmware shall maintain traceability between high-level requirements and implemented functionality.
- HL-MNT-002: The firmware shall support regression verification through stable command behavior.
- HL-MNT-003: The firmware shall provide version identification for build traceability.

---

### 13. Verification Requirements

- HL-VER-001: The firmware shall be verifiable through host interface testing and hardware-in-loop validation.
- HL-VER-002: The firmware shall support validation of communication, control, telemetry, safety, and persistence behaviors.
- HL-VER-003: The firmware shall ensure consistency between observed runtime behavior and defined requirements.

---

This high-level specification abstracts the detailed baseline requirements into system-level expectations, enabling clearer communication across firmware, hardware, and validation teams while preserving alignment with the implemented behavior.