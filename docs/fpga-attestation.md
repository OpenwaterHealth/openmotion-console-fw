# FPGA configuration attestation, the per-unit safety record and the safety envelope

Console firmware change closing the host-writable safety-state items of
penetration test PTR-2026-1-5 (§5.2.3, §5.3.2 and the follow-up with the
testers), CVA DC-00094 observation on host-writable state, and OI-12 of
ECO-secure-boot-1.2.0. Class B safety-relevant change (ARCH-64, DC-00052).

Source files: `Core/Src/fpga_attest.c`, `Core/Src/sha256.c`,
`Core/Inc/safety_envelope.h`, `Core/Src/safety_envelope.c`,
`Core/Src/safety_ufm_record.c`, `Core/Src/motion_cfg_keys.c`,
`Core/Src/motion_config.c`, `Core/Src/if_fpga_prog.c`,
`Core/Src/fpga_register_map.c`, `scripts/gen_fpga_digests.py`,
`scripts/check_release_symbols.sh`, `scripts/safety_ufm_record.py`.

## 1. What was wrong

The signed-firmware root of trust (bootloader 1.2.0, KMS-signed application)
did not cover three pieces of safety-relevant state:

1. `OW_FPGA_PROG` let any USB host erase and reprogram the seed, TA and
   safety (EE and OPT) MachXO2 FPGAs: configuration sector, UFM and feature
   row. The safety FPGA is the laser interlock.
2. Nothing verified at boot that the FPGA configuration present was the
   released bitstream.
3. The motion configuration record (CRC-16 only) carried TEC_TRIP, the EE/OPT
   thresholds and gains and the writable laser-driver register keys, and any
   USB host could write it.

## 2. What the firmware does now

### 2.1 Programming is absent from Release and Debug images

The programming service (`if_fpga_prog.c`) and the Lattice ECA programming
entry points (`XO2ECA_apiProgram`, `apiClearXO2`, `apiEraseFlash`,
`apiWriteUFM`, `XO2ECAcmd_EraseFlash`, `SRAMErase`, `CfgErase`,
`CfgWritePage`, `UFMErase`, `UFMWritePage`, `FeatureRowErase`,
`FeatureRowWrite`, `setDone`, `setUserCode`) are compiled only when the
build defines `OW_FACTORY_PROGRAMMING`. Only the CMake presets `Factory`
and `Factory-BareMetal` define it; `Release`, `Debug` and their bare-metal
variants never do, `clinical-build.yml` builds `Release` only, and CMake
refuses the option with a Release build type. The vendored-code change is
recorded in `Core/Src/LOCAL_CHANGES.md`.

In every other build an `OW_FPGA_PROG` packet is answered with `OW_ERROR`
and one data byte `OW_CODE_NOT_SUPPORTED` (0xFB), never a silent success.
`scripts/check_release_symbols.sh` runs `arm-none-eabi-nm` on the ELF and
fails CI (`build-firmware.yml` for every tag and branch build,
`clinical-build.yml` for the clinical image) if any programming symbol is
present. The read-only entry points stay in every build for the attestation.

### 2.2 Configuration attestation

`fpga_attest.c` checks the two safety devices on I2C mux 0 (channel 2 EE,
channel 3 OPT), which carry the laser interlock and bound emission on their
own (ARCH-68). The seed (channel 0) and TA (channel 1) drivers are not
attested; the report marks them "not checked" (status 0x40). Decision
2026-10-22: the safety devices are the ones whose integrity the interlock
depends on, and checking two instead of four halves the boot-time window. The
seed and TA designs cannot be changed from a Release image either (section
2.1). For each attested device it:

1. reads the Lattice device ID and the status register (a device that does
   not answer or reports BUSY is a read failure; this also keeps the vendored
   `openCfgIF` busy-wait from running past the 250 ms watchdog);
2. opens the configuration interface in **transparent** mode (user logic keeps
   running), resets the configuration address and reads
   `FPGA_ATTEST_CFG_PAGES` = 3198 pages of 16 bytes with
   `XO2ECAcmd_CfgReadPage`, streaming them through SHA-256
   (`sha256.c`, FIPS 180-4, verified against the NIST vectors in
   `host_tests/test_sha256.c`), then closes the interface;
3. compares the digest with the one compiled into the image
   (`fpga_attest_expected.h`, section 4);
4. for EE and OPT, reads the per-unit safety record from UFM pages 0-1
   (section 2.4) with `XO2ECA_apiReadBackUFM` and validates it against the
   envelope;
5. when the released bitstream contains the safety FPGA's own UFM loader
   (`ufm_config.v`; SBOM property `openwater:ufm-loader = "true"`), also
   reads the loader status register 0x31 over the user port and requires
   0x2F (record loaded and applied). 0.1.4 has no loader (the property is
   `"false"`), so the firmware's own validation is the whole check today; a
   loader release needs only the manifest change, not a code change.

The attested devices are a table in `fpga_attest.c` (device, expected
digest, holds a record, has a loader). Adding a device is one row plus its
digest from `gen_fpga_digests.py`.

The check runs in the background from the main loop, at most
`FPGA_ATTEST_BUDGET_MS` (12 ms) of bus time per iteration, on I2C1 at fast
mode (about 385 kHz, section 5), so the console
enumerates and answers commands while it runs. It starts once at the end of
initialisation and again on `OW_CMD_FPGA_ATTEST` with `reserved = 1`; a start
is refused with `OW_CODE_BUSY` while a run or the trigger is active, so it
never runs inside a scan.

**What is hashed.** The 3198 configuration-sector pages of the MachXO2-2000,
in page order, exactly the bytes the SDK's
`FpgaPageProgrammer` writes from the JEDEC fuse map (16-byte rows, MSB-first
per byte, from the `L` fields) and verifies after programming.

**What is excluded and why.**

| Region | Excluded | Why |
|---|---|---|
| USERCODE (`U` field) | yes | separate register, read with `readUserCode`; not part of the configuration pages. The released designs carry `H00000000`. |
| Feature row / FEABITS (`E` field) | yes | separate flash row, read with `FeatureRowRead`; not part of the configuration pages |
| Trace ID | yes | factory-programmed per device, read with `readTraceID` |
| Fuse rows 3198..3837 of the JEDEC file (UFM initialisation) | yes | the UFM is a separate sector that carries the per-unit safety record, validated on its own (section 2.4); `gen_fpga_digests.py` reports whether those rows are blank in the released artefacts (they are, for all three) |

The bench procedure to confirm that nothing inside the hashed pages varies
between programmings of the same `.jed` is in section 6.3; its result is
recorded there.

### 2.3 Interlock and reporting

Any digest mismatch, read failure, or a missing (blank), corrupt or
out-of-envelope safety record, or a loader bitstream that reports it did not
apply the record, sets `TRIG_INTERLOCK_FPGA_ATTEST` (0x40). The interlock
byte has no free bit (0x80 is the timer-start failure), so record faults
share 0x40; the attestation report says which: device status bit 0x08
(blank) or 0x04 (invalid), `ufm_status` (`safety_ufm_status_t`) and
`loader_status`. The bit is also set from reset until
the first run passes. While it is set `Trigger_Start()` refuses and the
refusal status byte carries the bit; after a FAILED run `Trigger_SetConfig()`
refuses too. The interlock is sticky until a run passes. A failed run stops
a running trigger, prints the per-device result on the debug UART and queues
one `{"type":"system","state":"error","msg":"FPGA attestation failed",
"device":..,"status":..}` message per faulty device (OW_CMD_MESSAGES).

Status reaches the application through:

- `OW_CMD_FPGA_ATTEST` (0x11), read-only: `fpga_attest_report_t` with the
  run state, progress, interlock flag, run count, and per device the status
  bits, UFM status, loader status (0xFF when not read), pages read, readback
  time, Lattice device ID, computed digest and expected digest. Report
  struct version 2: `loader_status` replaces the reserved byte and a blank
  record is a fault;
- `OW_CMD_BOOT_INFO` struct version 2: the interlock byte, the attestation
  state and a factory-image flag are appended to the v1 layout (a v1 parser
  still works: the SDK's `parse_boot_info` does not enforce the version);
- the `OW_CTRL_START_TRIG` refusal byte.

### 2.4 The per-unit safety record and the safety envelope

The safety parameters are calibrated per unit on the bench (WI-00015), so
they cannot all be compile-time constants. They are split three ways:

**The safety envelope (`Core/Inc/safety_envelope.h`).** Compiled into the
signed image; every value cites its source (WI-00015 as encoded in the SDK's
calibration modules, or the reset value in the FPGA design's `registers.v`
at the released tag). Each value is either a bound that the record and the
host setpoints can only tighten, never loosen, or a fixed value the firmware
writes itself:

| Parameter | Kind | Value | Source |
|---|---|---|---|
| EE/OPT `PULSE_WIDTH_UL` | max | 660 us | WI-00015 `MAX_PULSE_WIDTH_LIMIT_US` |
| EE/OPT `RATE_LL` | min | 23125 us (at most 43.2 Hz) | WI-00015 `DEFAULT_USER_CONFIG` |
| EE `DRIVE_CL` | max | 5000 mA | safety `registers.v` reset 0x0a80 |
| OPT `DRIVE_CL` | max | 7200 (x 1.86, optical ADC scale) | fleet maximum 7143; pending DC-00052 §9 |
| EE/OPT `PWM_CURRENT`, `CW_CURRENT` | max | 140 mA | safety `registers.v` reset 0x036b |
| `TA_PULSE_WIDTH` (operational) | max | 600 us | WI-00015 `MAX_TA_PULSE_WIDTH_US` |
| `TA_CURRENT_DRV` (operational) | max | 5000 mA | EE drive rating; pending DC-00052 §9 |
| `SEED_CW_GAIN`, `SEED_DDS_GAIN` (operational) | max | 160 | seed `registers.v`; DDS pending DC-00052 §9 |
| TEC trip | fixed | 40 C | WI-00015 `TEC_TRIP` |
| `TA_CURRENT_LIMIT`, `TA_PWM_MON_CL`, `TA_CW_MON_CL`, `TA_PERIOD`, `SEED_DDS_CL`, `SEED_CW_CL` | fixed | `registers.v` reset values | TA / seed designs |

**The per-unit safety record (safety FPGA UFM).** Each safety device holds a
32-byte record in UFM pages 0-1 (`safety_ufm_record.h`). The layout is the
one the safety FPGA's own loader reads (`openmotion-safety-fpga`
`src/ufm_config.v`, `docs/ufm_calibration.md`, branch
`calibration_support_merge`): magic "OW", layout 1, raw values of
`PULSE_WIDTH_LL/UL`, `RATE_LL`, `DRIVE_CL`, `PWM_CURRENT`, `CW_CURRENT`, and
a 16-bit truncated byte sum. `scripts/safety_ufm_record.py` builds it and
writes the Diamond `.mem` rows; its output is byte-identical to the FPGA
repository's `tools/make_ufm_record.py` (checked 2026-10-22). Values are
converted to raw rounding towards the conservative side (upper limits down,
the rate lower limit up), so 660 us becomes raw 2062 = 659.84 us. The record
is written only by a Factory image (section 2.1 and 6.4). A Release image
reads it at every attestation and requires magic, layout 1, zero reserved
bytes, the checksum, `PULSE_WIDTH_LL <= PULSE_WIDTH_UL`, and every value
inside the envelope. A VALID record is then:

- written into that device's limit registers verbatim and read back
  (`safety_envelope.c`, `motion_cfg_apply_settings()`, after every completed
  attestation and every configuration apply);
- the source of the unit's ceilings for the host setpoints:
  `TA_CURRENT_DRV <= EE DRIVE_CL` and `TA_PULSE_WIDTH <=` the smaller of the
  EE/OPT `PULSE_WIDTH_UL`, each also capped by the envelope
  (`motion_cfg_set_unit_ceiling()`). The ceilings apply to host
  configuration writes, to the stored-record apply and to the I2C
  pass-through.

A blank UFM (all 0x00 or 0xFF: the released bitstreams carry no UFM
initialisation), a corrupt record, a value outside the envelope, or (on a
loader bitstream) 0x31 other than 0x2F, is a latched
`TRIG_INTERLOCK_FPGA_ATTEST` (section 2.3). The laser cannot be armed, and
there is no fallback to the rated limits. A device without a valid record
gets no limit writes from the firmware: its registers keep the design reset
values, which are conservative, and the interlock keeps the laser off.

**Which values are calibrated per unit (WI-00015, checked 2026-10-22 against
`omotion/calibration/safety.py`, `safety_workflow.py`, `laser.py`).**
`EE_DRIVE_CL` / `OPT_DRIVE_CL` (the EE/OPT thresholds; `EE_THRESH` /
`OPT_THRESH` alias the same register 0x10) are the mean measured current
x 1.1 (EE) / x 1.3 (OPT). `EE/OPT_PULSE_WIDTH_UL` are the TA pulse width
x 1.1. Both are per unit, so they come from the record; the envelope only
bounds them. `TEC_TRIP` (40 C) and `EE/OPT_RATE_LL` (23125 us) are fixed
`DEFAULT_USER_CONFIG` values that the procedure never recalculates. They
stay compile-time: the TEC trip is fixed, and the rate lower limit is an
envelope minimum that the record carries. `PWM_CURRENT`, `CW_CURRENT` and
`PULSE_WIDTH_LL` are not touched by WI-00015.

**TEC trip.** The thermal-trip threshold is computed from
`SAFETY_TEC_TRIP_C` as the first statement of `main()`, before any
peripheral and before any record is read. The old failure mode (a parse
failure left `TEC_TRIP_VALUE` at 0 and disarmed the trip) is gone: no record
takes part. A Factory-image override may only lower it (20..40 C).

The driver clamps are written and read back by `motion_cfg_apply_settings()`
at boot, after every configuration write and after every completed
attestation. A failed write or read-back is `TRIG_INTERLOCK_LIMITS` (CFW-10,
unchanged). In a Factory image, safety keys in the stored configuration are
applied after the loads as bench overrides, within the same envelope.

The configuration record (schema v3) holds only **operational** keys and
the application's opaque `calibration` block:

| Key | Range | Source of the bound |
|---|---|---|
| `TA_CURRENT_DRV` | 0..5000 mA, and <= the unit's EE `DRIVE_CL` | WI-00015 default 5000; safety FPGA drive rating; per-unit record |
| `TA_PULSE_WIDTH` | 1..600 us, and <= the unit's EE/OPT `PULSE_WIDTH_UL` | WI-00015 `MAX_TA_PULSE_WIDTH_US`; per-unit record |
| `SEED_CW_GAIN` | 0..160 | seed CW current limit (registers.v) |
| `SEED_DDS_GAIN` | 0..160 | pending DC-00052 §9 |

A host write (`OW_CMD_USR_CFG`, reserved 1) naming a safety key is refused
with `OW_ERROR` + `OW_CODE_CFG_SAFETY_KEY` (0xFA); an unknown key with
`OW_CODE_CFG_UNKNOWN_KEY` (0xF9); a non-finite or out-of-range value with
`OW_CODE_CFG_RANGE` (0xF8). Nothing is stored on refusal. A safety key found
in a stored record (possible only from a Factory build) is ignored and
logged at apply. A v1/v2 record is migrated at load: operational and opaque
keys kept (an out-of-range value is dropped, never clamped), safety and
unknown keys dropped and counted on the log, and the record rewritten as v3.
A record that cannot be migrated is invalid as a whole (defaults,
`TRIG_INTERLOCK_CONFIG`), never half-applied.

The pass-through write `OW_CTRL_I2C_WR` follows the same ownership: a
firmware-owned register is refused ("firmware-owned safety register"), an
operational register is accepted only with a value inside its range
(including the per-unit ceiling); the
EE/OPT fault-clear control word stays usable with the trigger stopped
(CFW-08 behaviour kept).

## 3. Build and release plumbing

- `fpga-artefacts/<repo>/<tag>/<file>.jed`: the released JEDEC files, byte
  for byte the GitHub release assets of openmotion-safety-fpga 0.1.4,
  openmotion-seed-fpga 0.1.1 and openmotion-ta-fpga 1.1.0.
- `scripts/sbom-manifest.json` lists the three designs as components with
  `openwater:role = fpga-bitstream`, the repository, the release tag, the
  artefact name and SHA-256, the page count, the expected configuration
  digest and `openwater:ufm-loader` (whether the bitstream contains the
  safety FPGA's UFM loader; `"false"` for all three released bitstreams).
  They appear in the generated SBOM.
- `scripts/gen_fpga_digests.py` runs at CMake configure time, re-derives the
  digests from the artefacts and FAILS the build if a tag, an artefact, an
  artefact hash or a digest does not match the manifest. It writes
  `build/<preset>/generated/fpga_attest_expected.h` (digest, tag and
  `FPGA_ATTEST_UFM_LOADER_<device>` per design). Bumping an FPGA release
  is therefore a reviewed change to the manifest plus the new artefact
  (`--update-manifest` recomputes the hashes; `--fetch` downloads a missing
  asset with `gh`).
- Presets `Factory` and `Factory-BareMetal` (Debug build type +
  `OW_FACTORY_PROGRAMMING=ON`). A Factory image reports its version with a
  `-factory` suffix, prints a banner at boot and sets bit 0 of the boot-info
  `build` byte.
- Version: this is a Class B change; per SOP-00014 the next release is a
  minor bump from 1.8.x, proposed `1.9.0-rc.0` on `next` (the version string
  comes from the tag).

## 4. Expected digests (openmotion-console-fw at this change)

| Device(s) | Design | Tag | Artefact | Configuration digest (SHA-256 of 3198 pages) |
|---|---|---|---|---|
| EE, OPT | openmotion-safety-fpga | 0.1.4 | `Safety_impl1.jed` (sha256 `42bc3d2e2391ade0ceb3de06aa6bbbc8a2b822df3df78e5cf6f6e426dd9228b3`) | `c79ede6c19911c9efe95818851e021677bd091e7262edcfab5d3151d6b3b4ad9` |
| seed | openmotion-seed-fpga | 0.1.1 | `seed_driver_impl1.jed` (sha256 `c8168a481b72d54b868a1e843d49513aa6a2dd85680491671a4dc292d625c696`) | `7104d28d4c24bd7dfade8d4dc8353bc4b1d96f58061cd2abfaaec372fbff68b6` |
| TA | openmotion-ta-fpga | 1.1.0 | `TA_Driver_impl1.jed` (sha256 `64f1c5de410cc007075ddd344700d0eeee38f1c8b954e66891ad35e7ba5f1371`) | `d0f5482cee5e893706ef1cefecb42d13d709dc21d53cd14f94ecf77935b0bae0` |

All three JEDEC files have QF = 491264 (3838 rows); rows 3198..3837 are
blank in all three.

**Configuration-sector size: 3198 pages, not 3200 (found 2026-10-22).** The
first round of this change hashed 3200 pages, the size in the vendored
Lattice ECA table `XO2DevList`. A sequential configuration read continues
into the UFM after page 3197, so pages 3198 and 3199 of that read are UFM
pages 0 and 1. With a blank UFM that was invisible: JEDEC rows 3198/3199 are
blank too, and the digests matched. After the first safety records were
written on the bench, both devices reported a digest mismatch. Each mismatch
equals exactly SHA-256(released pages 0..3197 || that device's 32-byte
record), computed on the host, so the configuration flash itself was
unchanged. The firmware now hashes 3198 pages (manifest
`openwater:cfg-pages`). The vendored table is left as it is (programming and
`XO2ECA_apiReadBackCfg` use it): a programmer that writes all 3200 JEDEC rows
writes rows 3198/3199 into UFM pages 0/1. These rows are blank (zero) in
every released artefact and `SYSCONFIG CONFIGURATION=CFG` keeps them so, so
programming leaves the record as it is. `gen_fpga_digests.py` reports any
non-blank row beyond the configuration sector. Note: the `Safety_impl1.jed` tracked in the
openmotion-safety-fpga repository differs byte-wise from the release asset
(line endings); the release asset is the input.

## 5. Timing

I2C1 (mux 0 and the four MachXO2 configuration ports, nothing else) runs at
fast mode since this change: timing `0x10B71B27`, 60 MHz kernel clock,
tLOW 1.33 us, tHIGH 0.93 us, SCL 353-387 kHz for 50-300 ns rise time, every
fast-mode limit checked (tLOW, tHIGH, SDADEL, SCLDEL). It was `0x107075B0`
(about 100 kHz). I2C2 and I2C4 stay at 100 kHz: they also carry the
application FPGAs' fabric I2C slaves, which are not qualified at 400 kHz.

Bench measurement, console serial 338435643333, Release-policy (Debug preset)
image, 2026-10-22:

| Device | Pages | 100 kHz | 400 kHz |
|---|---|---|---|
| EE | 3200 (first round) | 13 604 ms | 3 104 - 3 189 ms |
| OPT | 3200 (first round) | 13 924 ms | 3 194 - 3 272 ms |
| seed, TA | - | 13 860 / 13 881 ms | not checked |
| Power-on to arm-able | | about 55 s (four devices) | about 6.5 s (two devices) |

At 400 kHz a page takes about 1.0 ms. Soak: ten back-to-back runs on request
with I2C health rescans (both buses) and TEC/telemetry reads between polls:
ten PASS, digests always equal, no I2C error on the debug UART, I2C health
`all_present` before and after; the HIL suite (safety state and I2C health)
passed at 400 kHz. FPGA programming from a Factory image has not been
exercised at 400 kHz yet; the configuration-port commands are the same.

Remaining options if 6.5 s is still too long: select the mux once per device
instead of once per command, and read several pages per command
(`LSC_READ_INCR_NV` page count); neither is done.

**Variation check result (section 6.3).** On this unit the configuration
pages of the safety devices, as read back through the configuration port,
hash to exactly the digest computed from the packed released JEDEC file, so
nothing inside the hashed pages differs between the artefact and a
factory-programmed device. The two-board / reprogram-twice diff has not been
run; it stays open in section 6.3 and does not change the design.

## 6. Bench procedures

### 6.1 Positive path (Release or Debug image)

1. Flash the image (signed for the bootloader on the unit, or bare-metal over
   the probe on a bench unit) and watch the debug UART: after
   "Initialize message queue" the attestation starts; each device logs
   `FPGA attestation: <dev> ok (id 0x..., 3198 pages, <ms> ms, UFM valid)`
   and the run ends with `FPGA attestation: PASS`. On a unit without
   provisioned records both devices log `FAIL (... UFM blank)`, the run
   fails and the laser stays interlocked: that is the expected state of an
   unprovisioned unit, not a fault of the image.
2. `pytest tests/test_safety_state_hil.py` with `OPENMOTION_HIL=1`:
   boot attestation passes (provisioned unit) or a missing/invalid record
   keeps the interlock (unprovisioned unit; the other test skips),
   request/busy behaviour, `OW_FPGA_PROG` refused, safety-key config write
   refused, TEC trip in force after a refused write.

**Bench results 2026-10-22, console 338435643333.**

- Unprovisioned (both UFMs blank), Release image:
  `test_missing_or_invalid_record_keeps_interlock` passes (digests match,
  device status 0x08, run FAIL, 0x40 set, start refused with 0x40); the
  request/busy, `OW_FPGA_PROG` refused, safety-key write refused and TEC
  trip tests pass.
- Provisioned with a Factory image (section 6.4): UFM erase (mode 0x08) plus
  pages 0-1 on EE and OPT, each read back byte-for-byte. Feature rows were
  unchanged (read before and after), and both devices reported DONE after
  REFRESH. Values (unit never WI-00015 calibrated, approved for the bench):
  EE DRIVE_CL 4999.68 mA (raw 0x0a80), OPT DRIVE_CL 7142.4 (raw 0x0f00),
  PW_UL 659.84 us, RATE_LL 23125.12 us, PWM/CW 140 mA.
- Provisioned, Release image (3198-page build): EE and OPT digests equal the
  released digest, UFM valid, run PASS, interlock clear; 2 982 - 3 129 ms
  per device. HIL suite: 5 passed, 2 skipped (the unprovisioned-unit test and
  the bench-reprogram test).
- After the provisioning, the first Release boot showed the TA driver's user
  port (mux 1 channel 4) not answering (I2C health `fpga=0x70`). TA, seed
  and EEPROM writes on mux 1 failed, and the post-attestation re-apply of
  the limit registers overran the 250 ms watchdog: a reset loop with the
  laser interlocked. A full power cycle cleared it (health `all_present`).
  Not root-caused. The safety design drives `TA_shutdown`, and REFRESH
  reconfigures EE/OPT, which may leave the TA device wedged until power-off.
  Two follow-ups: power-cycle after UFM provisioning (section 6.4 step 3),
  and keep the re-apply within the watchdog when the bus is stuck.

**Bootloader observation (bench bootloader 1.0.0, needs follow-up before a
tag).** Three times a Release image was refused at "VERIFY USER FW
SIGNATURE" (state HANDLE CRITICAL FAILURE, slot body cleared), although
`verify_firmware.py` accepts the file with the bootloader's public key. Each
time, DFU had been entered with `flash_firmware.py enter-dfu` from a running
Debug-build-type application (Debug or Factory preset). The same files
installed from DFU entered any other way (no valid firmware, or `enter-dfu`
from a running Release application) boot. Debug and Factory images were
never refused. An early guess that the image length mattered did not hold up
(the same unpadded file later booted). Not root-caused, and not checked on
bootloader 1.2.0 for that entry. On bootloader 1.2.0 (2026-10-26, rc.3
production image) `enter-dfu` from a running Release application followed by
the signed rc.3 slot image was accepted and booted, and an image signed with
the 1.0.0 key was refused before any erase (`errVENDOR`, installed image
intact); the failing case, entry from a Debug or Factory application, needs a
Debug-build-type image signed with the KMS key and has not been repeated. A
unit in that state drops to DFU and recovers by installing the image again. Separately, `flash_firmware.py leave` after
`enter-dfu` with no download leaves the unit in DFU (the slot is intact):
install an image to leave.

### 6.2 Negative path (bench unit only, never a unit that will ship)

**Caution (bench incident 2026-10-22).** Use a modified copy of the released
`.jed` (one flipped fuse in an `L` field) and keep its `E` field. Do NOT use
an older release as the "different" bitstream: openmotion-seed-fpga 1.0.3
predates the "enable i2c updates" release and its feature row disables the
I2C configuration port. Programming it through the Factory image left the
seed device of the bench console unreachable over I2C (configuration and
user port both silent, also after a power cycle; recovery needs the device's
JTAG). With that device silent the Release-policy image reports the seed as
READ_FAIL and the run as FAILED. That was the read-failure route at a time
when all four devices were attested; since the 2026-10-22 decision the seed
is "not checked", so a seed fault no longer exercises this path. The Factory
image now refuses a FEABITS value with that bit set (`if_fpga_prog.c`,
FEATROW_WRITE), and the SDK programmer should refuse it too.

1. Build and flash the `Factory` preset.
2. With the SDK `FpgaPageProgrammer`, program ONE of the two attested
   safety devices (EE, mux 0 channel 2, or OPT, channel 3) with a bitstream
   that differs from the released one. Do not use the seed or TA device:
   they are not attested (section 2.2), so a change there is reported "not
   checked" and the run still passes. A different USERCODE is NOT enough,
   since USERCODE is excluded. Flip one fuse in a copy of the released
   `.jed`, inside the first 3198 pages (fuse index < 3198 x 128), keep the
   `E` field (feature row) and adjust the `C` checksum. A fuse in a blank
   row past the design's last used row leaves the design loadable.
3. The programmer erases the whole device (`ERASE_ALL`), the UFM included,
   so re-provision that device's safety record (section 6.4 step 3) before
   leaving the Factory image. Otherwise the device also reports `UFM blank`
   and the test does not isolate the digest route.
4. Flash the Release/Debug image and power-cycle the console fully.
   Expected: the attestation logs `FAIL` for that device with status
   `DIGEST | UFM_VALID` (0x11), the other safety device passes,
   `OW_CMD_FPGA_ATTEST` shows the digest mismatch, boot-info reports state
   3 with the 0x40 interlock, `OW_CTRL_START_TRIG` is refused with 0x40 in
   the status byte, and `tests/test_safety_state_hil.py` with
   `OPENMOTION_HIL_REPROGRAM=1` passes
   `test_modified_bitstream_sets_interlock_and_blocks_arming`. The three
   tests that expect a correctly programmed unit fail in this state by
   design (boot attestation, attestation on request, `OW_FPGA_PROG`
   refusal, which also checks the digests).
5. Recovery: Factory image, program the released `.jed` from
   `fpga-artefacts/`, re-provision the UFM record (section 6.4), power-cycle,
   flash the Release image; the attestation passes again.

**Result (2026-10-26, bench console 338435643333, firmware `next`
b51e36b, Factory-BareMetal then Release-BareMetal over SWD).** EE was
programmed with a copy of safety 0.1.4 `Safety_impl1.jed` with fuse 384000
(page 3000, a blank row; the design's last used row is 1597) set to 1, `C`
field 0x8947 -> 0x8948, feature row and FEABITS identical. Programming took
205 s, CFG verify passed on 3838 pages, and status after REFRESH was
0x00080100 (DONE set, design loaded). The EE record was re-provisioned with
the 2026-10-22 bench values and read back. After a full power cycle the
Release image reported EE digest `3d15e453...46213d8` against the expected
`c79ede6c...3b4ad9`, EE status 0x11, OPT PASS with `UFM valid`, interlock
0x40 set, and the trigger start was refused with 0x40.
`test_modified_bitstream_sets_interlock_and_blocks_arming` passed (TC-FPGA-009
by the digest-mismatch route), together with the config-refusal, TEC-trip and
I2C health tests (TA answered after the power cycle).

### 6.3 Variation check (same `.jed` programmed twice)

Purpose: establish empirically that no byte inside the hashed pages varies
between programmings of the same bitstream. Procedure: on a bench unit
running a Factory image, read the 3198 pages of one safety device (EE or OPT)
(`OW_CMD_FPGA_PROG_CFG_RESET` + `CFG_READ_PAGE` x 3198), program the same
released `.jed` with the SDK programmer (it erases the UFM: re-provision the
record afterwards, section 6.4), read the pages again, and diff; then
compare with a second board if one is available. Result: not run yet (see
section 5 for what the single-unit attestation already establishes).
The design of the check does not depend on the outcome: USERCODE, feature row
and trace ID are outside the pages, and the attestation on the first unit
(section 5) confirms the pages read back equal the packed JEDEC rows.

### 6.4 Factory process (manufacturing and RMA bench)

1. Install the Factory image of the release. The clinical build of every
   tag stores it with the release:
   `gs://openwater-firmware-artifacts/openmotion-console-fw/<tag>/openmotion-console-fw-<tag>-factory-baremetal.{bin,hex}`
   (preset `Factory-BareMetal`, same commit as the clinical image, listed in
   `SHA256SUMS` and the manifest). It is unsigned and has no bootloader:
   flash it at 0x08000000 over SWD on an unlocked unit
   (`BENCH-ERASE-PROTECTED-UNIT.md` for a unit under a Release bootloader).
   The image version carries `-factory`; the boot banner says so; boot-info
   bit 0 of `build` is set. A local build (`cmake --preset Factory-BareMetal`,
   or `Factory` signed with a local test key against a Debug bootloader) is
   for development only. A Factory image must never be signed with the
   clinical key.
2. Program the FPGAs with the released artefacts from `fpga-artefacts/`
   (SDK `FpgaPageProgrammer.program_from_jedec`).
3. Run WI-00015 laser and safety calibration. On a Factory image the
   calibration tooling may still write the safety keys into the
   configuration (they are accepted within the hard bounds as overrides);
   the resulting per-unit values are then written into each safety device's
   UFM record: `scripts/safety_ufm_record.py build --device EE ...` and
   `--device OPT ...` (engineering units in; raw, conservatively rounded
   values out; the tool refuses anything outside the envelope), then
   `fpga_prog_open` (offline) / UFM erase / `ufm_reset` / write UFM pages 0
   and 1 / `close` through the SDK. On a Diamond bench the `--mem` output is
   programmed with "FLASH UFM Erase" then "FLASH UFM Program,Verify". The
   configuration-flash update must use "FLASH CFG Erase,Program,Verify",
   which leaves the UFM alone (`docs/ufm_calibration.md` of the safety
   design). Read the record back (`safety_ufm_record.py decode --device ...`).
   Then power-cycle the console fully (bench observation 2026-10-22,
   section 6.1).
4. Install the Release (clinical) image of the same tag. Its attestation must report
   `UFM valid` for EE and OPT and the run PASS; the stored
   configuration is migrated to v3 (safety keys dropped) at first boot. A
   unit that reports `UFM blank` or an invalid record goes back to step 3.
5. RMA bench recovery of a unit whose attestation fails: follow 6.2 step 4.
   The locked-bootloader recovery of WI "RMA Locked Bootloader Recovery"
   (SW 62304 Compliance folder) applies first when the unit is at RDP 1.

### 6.5 Deferred: identity fields and a signed safety record

**TODO (FPGA change, not in this firmware change).** The record carries no
unit serial number, calibration date or operator id, and no device byte. Any
field added inside the 32 bytes changes the layout `ufm_config.v` reads and
checksums, so it belongs to an openmotion-safety-fpga change (new layout
version, loader update) followed by a firmware layout bump. Until then, a
record copied from one unit to another would validate if its values are
inside the envelope. That cannot loosen the envelope; it can only give a
unit another unit's tighter-than-envelope limits. Binding the record to the
unit is part of the TODO.

**Deferred hardening (later release, not implemented).** Sign the record at
the calibration station with an Openwater calibration key (ECDSA P-256, the
public key compiled into the firmware), so that neither a Factory image nor
a flash-level write to the UFM can forge it. Deferred because it needs:

- key custody for a calibration key (Google Cloud KMS like the firmware key,
  `PLAN-rdp1-key-custody.md`);
- a signing step inside WI-00015;
- room for a 64-byte signature, which does not fit the 32-byte layout the
  FPGA loader reads. It would need a second UFM area that the loader
  ignores, plus a firmware layout bump.

## 7. Residual risk after this change (for the pen-test response)

An attacker with USB access to a console running a Release image can still:

- write the operational keys `TA_CURRENT_DRV`, `TA_PULSE_WIDTH`,
  `SEED_CW_GAIN`, `SEED_DDS_GAIN` within the firmware's validated ranges
  (the envelope, lowered by the unit's safety record), and the opaque
  `calibration` block the firmware does not interpret;
- write the same four registers over `OW_CTRL_I2C_WR` within the same
  ranges, and clear the EE/OPT latched faults with the trigger stopped;
- read everything the protocol exposes (no confidentiality claim is made).

The attacker cannot: change TEC_TRIP, any EE/OPT limit, driver clamp or
control word (compiled in or from the factory record, refused on both the
configuration and the pass-through path, re-asserted and read back at boot
and after every configuration write); erase, program or alter any FPGA
configuration, UFM (and so the per-unit safety record) or feature row (the
code is absent from the image); or run a modified safety
FPGA bitstream undetected (the EE and OPT digests are compared at boot and on
request against values inside the signed image, and the laser cannot be
armed until they match). The seed and TA driver designs are not attested: a
change to them made outside the console (JTAG, or a Factory image) would not
be detected by the console, but the attested safety FPGAs still bound pulse
width, rate and drive current independently.
The safety FPGAs' own hardware inhibit (ARCH-68) is unchanged and bounds
emission independently.

**Why a 16-bit checksum is acceptable on the safety record.** The checksum
is error detection only; it is the layout the FPGA loader reads. Integrity
rests elsewhere. The only write path is a Factory image, which is
Openwater-signed and never fielded, so forging the record needs either
flash-level access to the FPGA (JTAG on the board) or a signed Factory
image. And whatever the record holds, every value is bounded by the envelope
compiled into the signed Release image: a forged record can make a unit more
conservative than the envelope, never less. The same argument covers the
operational configuration page (CRC-16) for the operational keys.

What this does NOT cover: physical access to the FPGA JTAG/SPI pins or to
the SWD port on a unit without RDP (bootloader 1.2.0 sets RDP 1); a
compromised signing key (KMS, DC-00052 §12); the SDK and application, which
must stop writing the safety-owned registers (section 8).

## 8. Host-side changes required (openmotion-sdk, openmotion-test-app, bloodflow app)

Not part of this repository, listed for the coordinated release:

- `omotion/laser.py apply_laser_power`: stop writing the EE/OPT limit
  registers, `TA_CURRENT_LIMIT`, monitor limits and `DRIVE_CL` from the
  fpga-map JSON and user config over `OW_CTRL_I2C_WR`; the console refuses
  them and the read-back comparison then fails laser initialisation.
- `MotionConsole.write_config` callers: drop `TEC_TRIP`, `OPT_THRESH`,
  `EE_THRESH`, `OPT_GAIN`, `EE_GAIN`, `EE_*`, `OPT_*` from the written
  record; the console answers `OW_CODE_CFG_SAFETY_KEY`. The test-app TEC
  trip editor and the safety-calibration workflow are the known writers.
- `omotion/config.py`: mirror `OW_CMD_FPGA_ATTEST = 0x11`,
  `OW_CODE_CFG_RANGE/UNKNOWN_KEY/SAFETY_KEY`, `OW_CODE_NOT_SUPPORTED`,
  `OW_CODE_BUSY`, `TRIG_INTERLOCK_FPGA_ATTEST = 0x40`; parse boot-info v2
  and the attestation report; show the interlock in the application.
- `omotion/config.py` / attestation parser: report struct version 2
  (`loader_status`, blank record is a fault); `safety_ufm_status_t` values
  0..7 (4 = FPGA loader on defaults).
- Calibration tooling: a Factory image on the bench, and the UFM record
  write at the end of WI-00015 (section 6.4), UFM pages 0-1 in the FPGA
  loader layout, never the plain "FLASH Erase,Program,Verify" operation for
  configuration updates (it wipes the UFM).
