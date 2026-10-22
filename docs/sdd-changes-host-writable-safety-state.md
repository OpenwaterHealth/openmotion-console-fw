# DC-00078 Software Detailed Design: text changes for the host-writable safety-state remediation

Proposed replacement text for the controlled document (SW 62304 Compliance /
DC-00078), to be applied under document control together with the PR. Row
and section numbers refer to DC-00078 as of 2026-10-22. See
docs/fpga-attestation.md for the design itself.

## 4.1 Console firmware, module allocation

**ARCH-73 Seed-configuration FPGA programming service** (replace):
Service only, present in Factory images only (build option
`OW_FACTORY_PROGRAMMING`, preset `Factory`). Implements the vendor
programming sequence: open the configuration interface, erase, write and
read back configuration and user-flash pages, feature row, set done, refresh,
status. It is refused while the configuration attestation reads or the
trigger runs. A Release or Debug image contains none of this code and
answers the programming class with a defined "not supported" error; the
release pipeline checks the image for programming symbols.

**ARCH-76 Thermal control and indication** (replace the threshold sentences):
The trip threshold is a compile-time constant of the firmware
(`SAFETY_TEC_TRIP_C`), converted from temperature to the sensed quantity as
the first action of the executive, before any peripheral or the
configuration record is read. It does not depend on the configuration
record. A Factory image may override it from the record within hard bounds.

**ARCH-78 Configuration manager** (replace the last two sentences):
Applying the record classifies every key: operational keys (the laser
operating point) are range-checked against firmware ceilings and loaded;
safety keys and unknown keys are ignored and logged. A host write is
validated against the allow-list before anything is stored: a safety key, an
unknown key or an out-of-range value is refused with a defined error code
and the stored record is unchanged. Records of schema v1/v2 are migrated to
v3 at load (safety and unknown keys dropped, out-of-range operational values
dropped) or rejected as a whole.

**ARCH-83 Laser driver FPGA register abstraction** (append):
Each writable register has an owner: firmware-owned (safety limits, driver
clamps, static control words) or operational. Pass-through writes to a
firmware-owned register are refused in Release and Debug images; writes to
an operational register are accepted only within its permitted range.

**ARCH-125 Safety setpoint loader (console part)** (replace):
Loads the laser-safety limits into the EE and OPT limit registers and the
driver clamps into the amplifier and seed registers at boot, after every
configuration write and after every completed configuration attestation. The
values are compile-time constants of the firmware (ratings and WI-00015
limits), replaced per device by a valid per-unit calibration record read
from the safety FPGA user flash. Every write is read back; a failed write or
read-back raises a fault event and the trigger is refused. The operating
point (amplifier current, pulse width, seed gains) is then loaded from the
configuration record within ceilings the firmware owns, lowered by the
per-unit record where it sets one. The host no longer takes part in the
limit load.

**New row, ARCH-126 FPGA configuration attestation** (add; allocate the
number in the architecture): For each of the four MachXO2 devices, reads the
configuration flash through the configuration interface in transparent mode,
page by page from the main loop, hashes it with SHA-256 and compares the
digest with the value compiled into the signed image from the released
bitstream artefacts; for the safety devices also reads and validates the
per-unit calibration record (magic, layout, device, checksum, bounds). Runs
once after initialisation and on host request, never during a scan. A
mismatch, read failure or invalid record sets a sticky interlock that refuses
trigger start (and, after a failed run, trigger configuration) until a run
passes, raises a fault event and is reported in the status, boot-information
and attestation commands.

## 5.2 Configuration record (replace the console JSON paragraph)

The console JSON object (schema v3) holds the laser operating point for the
amplifier and seed drivers (`TA_CURRENT_DRV`, `TA_PULSE_WIDTH`,
`SEED_CW_GAIN`, `SEED_DDS_GAIN`), each with a permitted range the firmware
enforces, and the application's calibration block, which the firmware
stores but does not interpret. The laser safety limits, the driver clamps and
the thermal trip threshold are not part of the record: they are constants of
the firmware image (`safety_params.h`), with per-unit safety limits held in
the safety FPGA user flash record (section 5.4).

## 5.4 Safety FPGA user-flash record (new section)

Layout version 1, 32 bytes at UFM pages 600 and 601 of each safety device:
magic `OWSR`, layout, device, pulse-width upper limit, rate lower limit,
drive-current limit, PWM and CW current limits, per-unit ceilings for the
amplifier current and pulse width, reserved, CRC-16/CCITT-FALSE. Written by
a Factory image at the end of WI-00015 safety calibration; read and validated
by the attestation. A blank user flash means "no record": the firmware
ratings apply.

## 7.2 Trigger and interlock state machine (append to the Interlocked row)

"...; configuration attestation pending or failed (clears when a run
passes)". The start refusal status byte gains bit 0x40.

## 7.4 Safety limit load and verification (replace)

- At console boot, after every configuration write and after every
  completed attestation, the setpoint loader writes the compile-time limits
  (or the per-unit record values) to the EE and OPT registers and the driver
  clamps, and reads every register back.
- The SDK no longer writes the limit registers at connection; its writes to
  firmware-owned registers are refused. It reads the registers and the
  attestation report to show the state.

## Appendix A, opcodes

- A.1: 0xEB OW_FPGA_PROG: "Request: FPGA programming service (Factory images
  only; otherwise OW_ERROR + 0xFB)".
- A.2 console: add 0x11 FPGA_ATTEST "FPGA configuration attestation report;
  reserved 1 starts a run".
- Error codes (new table or note): 0xF8 CFG_RANGE, 0xF9 CFG_UNKNOWN_KEY,
  0xFA CFG_SAFETY_KEY, 0xFB NOT_SUPPORTED, 0xFC BUSY as the single data byte
  of an OW_ERROR reply.

## Other documents

- DC-00052 §9: the bounds of `safety_params.h` (rating per register, source
  per value) and the per-unit record; §12 item 2 unchanged.
- SAV (Security Architecture Views) §7.2: OI-12 closure; §5.1 firmware rows:
  the FPGA bitstreams enter the root of trust through the digest compiled
  into the signed image.
- DC-00094 (CVA) observation on host-writable state: closed by this change;
  residual risk per docs/fpga-attestation.md §7.
- PTR-2026-1-5 §5.2.3 and §5.3.2: the response is docs/fpga-attestation.md
  §7; the CRC on the record remains error detection only, integrity is the
  signed image.
