# FPGA configuration attestation and firmware-owned safety parameters

Console firmware change closing the host-writable safety-state items of
penetration test PTR-2026-1-5 (§5.2.3, §5.3.2 and the follow-up with the
testers), CVA DC-00094 observation on host-writable state, and OI-12 of
ECO-secure-boot-1.2.0. Class B safety-relevant change (ARCH-64, DC-00052).

Source files: `Core/Src/fpga_attest.c`, `Core/Src/sha256.c`,
`Core/Inc/safety_params.h`, `Core/Src/safety_params.c`,
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
   `FPGA_ATTEST_CFG_PAGES` = 3200 pages of 16 bytes with
   `XO2ECAcmd_CfgReadPage`, streaming them through SHA-256
   (`sha256.c`, FIPS 180-4, verified against the NIST vectors in
   `host_tests/test_sha256.c`), then closes the interface;
3. compares the digest with the one compiled into the image
   (`fpga_attest_expected.h`, section 4);
4. for EE and OPT, reads the per-unit UFM record (section 2.4) with
   `XO2ECA_apiReadBackUFM` and validates it.

The check runs in the background from the main loop, at most
`FPGA_ATTEST_BUDGET_MS` (12 ms) of bus time per iteration, on I2C1 at fast
mode (about 385 kHz, section 5), so the console
enumerates and answers commands while it runs. It starts once at the end of
initialisation and again on `OW_CMD_FPGA_ATTEST` with `reserved = 1`; a start
is refused with `OW_CODE_BUSY` while a run or the trigger is active, so it
never runs inside a scan.

**What is hashed.** The 3200 configuration-sector pages of the MachXO2-2000
(Lattice ECA `XO2DevList`), in page order, exactly the bytes the SDK's
`FpgaPageProgrammer` writes from the JEDEC fuse map (16-byte rows, MSB-first
per byte, from the `L` fields) and verifies after programming.

**What is excluded and why.**

| Region | Excluded | Why |
|---|---|---|
| USERCODE (`U` field) | yes | separate register, read with `readUserCode`; not part of the configuration pages. The released designs carry `H00000000`. |
| Feature row / FEABITS (`E` field) | yes | separate flash row, read with `FeatureRowRead`; not part of the configuration pages |
| Trace ID | yes | factory-programmed per device, read with `readTraceID` |
| Fuse rows 3200..3837 of the JEDEC file (UFM initialisation) | yes | the UFM carries the per-unit safety record; `gen_fpga_digests.py` reports whether those rows are blank in the released artefacts (they are, for all three) |

The bench procedure to confirm that nothing inside the hashed pages varies
between programmings of the same `.jed` is in section 6.3; its result is
recorded there.

### 2.3 Interlock and reporting

Any digest mismatch, read failure or invalid UFM record sets
`TRIG_INTERLOCK_FPGA_ATTEST` (0x40). The bit is also set from reset until
the first run passes. While it is set `Trigger_Start()` refuses and the
refusal status byte carries the bit; after a FAILED run `Trigger_SetConfig()`
refuses too. The interlock is sticky until a run passes. A failed run stops
a running trigger, prints the per-device result on the debug UART and queues
one `{"type":"system","state":"error","msg":"FPGA attestation failed",
"device":..,"status":..}` message per faulty device (OW_CMD_MESSAGES).

Status reaches the application through:

- `OW_CMD_FPGA_ATTEST` (0x11), read-only: `fpga_attest_report_t` with the
  run state, progress, interlock flag, run count, and per device the status
  bits, UFM status, pages read, readback time, Lattice device ID, computed
  digest and expected digest;
- `OW_CMD_BOOT_INFO` struct version 2: the interlock byte, the attestation
  state and a factory-image flag are appended to the v1 layout (a v1 parser
  still works: the SDK's `parse_boot_info` does not enforce the version);
- the `OW_CTRL_START_TRIG` refusal byte.

### 2.4 Safety parameters are firmware-owned

`Core/Inc/safety_params.h` holds every laser-safety parameter with its
source (WI-00015 as encoded in the SDK's calibration modules, or the reset
value in the FPGA design's `registers.v` at the released tag). They are
compiled into the signed image and in force from reset:

- TEC_TRIP: the thermal-trip threshold is computed from `SAFETY_TEC_TRIP_C`
  as the first statement of `main()`, before any peripheral and before the
  configuration record is read. The old failure mode (parse failure left
  `TEC_TRIP_VALUE` at 0 and disarmed the trip) is gone: the record no longer
  takes part.
- The EE/OPT limit registers (`PULSE_WIDTH_LL/UL`, `RATE_LL`, `DRIVE_CL`,
  `PWM_CURRENT`, `CW_CURRENT`) and the driver clamps (`TA_CURRENT_LIMIT`,
  `TA_PWM_MON_CL`, `TA_CW_MON_CL`, `TA_PERIOD`, `SEED_DDS_CL`, `SEED_CW_CL`)
  are written and read back by `motion_cfg_apply_settings()` at boot, after
  every configuration write and after every completed attestation
  (`safety_params.c` builds the load list). A failed write or read-back is
  `TRIG_INTERLOCK_LIMITS` (CFW-10, unchanged).

The configuration record (schema v3) holds only **operational** keys and
the application's opaque `calibration` block:

| Key | Range | Source of the bound |
|---|---|---|
| `TA_CURRENT_DRV` | 0..5000 mA | WI-00015 default 5000; safety FPGA drive rating |
| `TA_PULSE_WIDTH` | 1..600 us | WI-00015 `MAX_TA_PULSE_WIDTH_US` |
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
operational register is accepted only with a value inside its range; the
EE/OPT fault-clear control word stays usable with the trigger stopped
(CFW-08 behaviour kept).

**Per-unit limits (safety FPGA UFM record).** WI-00015 safety calibration
derives per-unit values (`DRIVE_CL` = measured x 1.3 / 1.1, `PULSE_WIDTH_UL` =
TA pulse width x 1.1). Those live in the UFM of each safety device as a
32-byte record (`safety_ufm_record.h`: magic `OWSR`, layout 1, device,
limits, CRC-16/CCITT-FALSE), written only by a Factory image. A VALID record
replaces the compile-time ratings for that device and may lower the
operational ceilings for `TA_CURRENT_DRV` / `TA_PULSE_WIDTH`; a BLANK UFM
means "no record": the ratings apply and the status is reported, not
interlocked; a record that is present but corrupt or out of the hard bounds
is `TRIG_INTERLOCK_FPGA_ATTEST`. The released safety design has the UFM
disabled in its EFB and no UFM initialisation (`efb_i2c.v`), so a unit that
has never been provisioned reads BLANK. `scripts/safety_ufm_record.py`
builds and decodes the record on the host.

## 3. Build and release plumbing

- `fpga-artefacts/<repo>/<tag>/<file>.jed`: the released JEDEC files, byte
  for byte the GitHub release assets of openmotion-safety-fpga 0.1.4,
  openmotion-seed-fpga 0.1.1 and openmotion-ta-fpga 1.1.0.
- `scripts/sbom-manifest.json` lists the three designs as components with
  `openwater:role = fpga-bitstream`, the repository, the release tag, the
  artefact name and SHA-256, the page count and the expected configuration
  digest. They appear in the generated SBOM.
- `scripts/gen_fpga_digests.py` runs at CMake configure time, re-derives the
  digests from the artefacts and FAILS the build if a tag, an artefact, an
  artefact hash or a digest does not match the manifest. It writes
  `build/<preset>/generated/fpga_attest_expected.h`. Bumping an FPGA release
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

| Device(s) | Design | Tag | Artefact | Configuration digest (SHA-256 of 3200 pages) |
|---|---|---|---|---|
| EE, OPT | openmotion-safety-fpga | 0.1.4 | `Safety_impl1.jed` (sha256 `42bc3d2e2391ade0ceb3de06aa6bbbc8a2b822df3df78e5cf6f6e426dd9228b3`) | `dcf5f8ef64f80602835f40dbae898f28480989dcb631ff37bc2dc32cc85886be` |
| seed | openmotion-seed-fpga | 0.1.1 | `seed_driver_impl1.jed` (sha256 `c8168a481b72d54b868a1e843d49513aa6a2dd85680491671a4dc292d625c696`) | `8e71816fbabafe9ec8cd86c133e7522c17bb979859cd6a43ff04034a54d6d650` |
| TA | openmotion-ta-fpga | 1.1.0 | `TA_Driver_impl1.jed` (sha256 `64f1c5de410cc007075ddd344700d0eeee38f1c8b954e66891ad35e7ba5f1371`) | `628fc1244b9c429550a366e5974f468b460fd85d723fa42149c7073546f55de6` |

All three JEDEC files have QF = 491264 (3838 rows); rows 3200..3837 are
blank in all three. Note: the `Safety_impl1.jed` tracked in the
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
| EE | 3200 | 13 604 ms | 3 104 - 3 189 ms |
| OPT | 3200 | 13 924 ms | 3 194 - 3 272 ms |
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
   `FPGA attestation: <dev> ok (id 0x..., 3200 pages, <ms> ms, UFM <status>)`
   and the run ends with `FPGA attestation: PASS`.
2. `pytest tests/test_safety_state_hil.py` with `OPENMOTION_HIL=1`:
   boot attestation passes, request/busy behaviour, `OW_FPGA_PROG` refused,
   safety-key config write refused, TEC trip in force after a refused write.

### 6.2 Negative path (bench unit only, never a unit that will ship)

**Caution (bench incident 2026-10-22).** Use a modified copy of the released
`.jed` (one flipped fuse in an `L` field) and keep its `E` field. Do NOT use
an older release as the "different" bitstream: openmotion-seed-fpga 1.0.3
predates the "enable i2c updates" release and its feature row disables the
I2C configuration port. Programming it through the Factory image left the
seed device of the bench console unreachable over I2C (configuration and
user port both silent, also after a power cycle; recovery needs the device's
JTAG). With that device silent the Release-policy image reports the seed as
READ_FAIL, the run as FAILED and refuses the trigger with 0x40, which is the
negative path of TC-FPGA-009 by the read-failure route; the digest-mismatch
route stays to be run after the JTAG recovery. The Factory image
now refuses a FEABITS value with that bit set (`if_fpga_prog.c`,
FEATROW_WRITE), and the SDK programmer should refuse it too.

1. Build and flash the `Factory` preset.
2. With the SDK `FpgaPageProgrammer`, program ONE device (the seed driver is
   the least consequential) with a bitstream that differs from the released
   one (for example the same design with a different USERCODE is NOT enough,
   since USERCODE is excluded; flip one fuse in a copy of the `.jed` so the
   design still loads, or program a previous release tag).
3. Flash the Release/Debug image again. Expected: the attestation logs
   `FAIL` for that device, `OW_CMD_FPGA_ATTEST` shows the digest mismatch,
   boot-info reports state 3 with the 0x40 interlock, `OW_CTRL_START_TRIG`
   is refused with 0x40 in the status byte, and
   `tests/test_safety_state_hil.py` with `OPENMOTION_HIL_REPROGRAM=1` passes.
4. Recovery: Factory image, program the released `.jed` from
   `fpga-artefacts/`, re-provision the UFM record (section 6.4), flash the
   Release image; the attestation passes again.

### 6.3 Variation check (same `.jed` programmed twice)

Purpose: establish empirically that no byte inside the hashed pages varies
between programmings of the same bitstream. Procedure: on a bench unit
running a Factory image, read the 3200 pages of the seed device
(`OW_CMD_FPGA_PROG_CFG_RESET` + `CFG_READ_PAGE` x 3200), program the same
released `.jed` with the SDK programmer, read the pages again, and diff; then
compare with a second board if one is available. Result: not run yet (see
section 5 for what the single-unit attestation already establishes).
The design of the check does not depend on the outcome: USERCODE, feature row
and trace ID are outside the pages, and the attestation on the first unit
(section 5) confirms the pages read back equal the packed JEDEC rows.

### 6.4 Factory process (manufacturing and RMA bench)

1. Build `cmake --preset Factory && cmake --build build/Factory`. The image
   version carries `-factory`; the boot banner says so; boot-info bit 0 of
   `build` is set. It is signed and installed like any bench image (local
   test key against a Debug bootloader) and must never be signed with the
   clinical key.
2. Program the FPGAs with the released artefacts from `fpga-artefacts/`
   (SDK `FpgaPageProgrammer.program_from_jedec`).
3. Run WI-00015 laser and safety calibration. On a Factory image the
   calibration tooling may still write the safety keys into the
   configuration (they are accepted within the hard bounds as overrides);
   the resulting per-unit values are then written into each safety device's
   UFM record: `scripts/safety_ufm_record.py build --device EE ...` and
   `--device OPT ...`, then `fpga_prog_open` (offline) / `ufm_reset` /
   write pages 600 and 601 / `close` through the SDK, or an equivalent
   transparent-mode UFM write.
4. Install the Release (clinical) image. Its attestation must report
   `UFM valid` for EE and OPT and the run PASS; the stored
   configuration is migrated to v3 (safety keys dropped) at first boot.
5. RMA bench recovery of a unit whose attestation fails: follow 6.2 step 4.
   The locked-bootloader recovery of WI "RMA Locked Bootloader Recovery"
   (SW 62304 Compliance folder) applies first when the unit is at RDP 1.

## 7. Residual risk after this change (for the pen-test response)

An attacker with USB access to a console running a Release image can still:

- write the operational keys `TA_CURRENT_DRV`, `TA_PULSE_WIDTH`,
  `SEED_CW_GAIN`, `SEED_DDS_GAIN` within the firmware's validated ranges,
  and the opaque `calibration` block the firmware does not interpret;
- write the same four registers over `OW_CTRL_I2C_WR` within the same
  ranges, and clear the EE/OPT latched faults with the trigger stopped;
- read everything the protocol exposes (no confidentiality claim is made).

The attacker cannot: change TEC_TRIP or any EE/OPT limit, driver clamp or
control word (compiled in, refused on both the configuration and the
pass-through path, re-asserted and read back at boot and after every
configuration write); erase, program or alter any FPGA configuration, UFM or
feature row (the code is absent from the image); or run a modified safety
FPGA bitstream undetected (the EE and OPT digests are compared at boot and on
request against values inside the signed image, and the laser cannot be
armed until they match). The seed and TA driver designs are not attested: a
change to them made outside the console (JTAG, or a Factory image) would not
be detected by the console, but the attested safety FPGAs still bound pulse
width, rate and drive current independently.
The safety FPGAs' own hardware inhibit (ARCH-68) is unchanged and bounds
emission independently.

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
- Calibration tooling: a Factory image on the bench, and the UFM record
  write at the end of WI-00015 (section 6.4).
