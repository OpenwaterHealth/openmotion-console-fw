# Local changes to vendored code in Core/

Vendor code stays vendor code: these files are third-party (SOUP, recorded in
`scripts/sbom-manifest.json`) and every local change is listed here with its
reason.

## Lattice MachXO2 Embedded Configuration Access API (`XO2_api.c/.h`, `XO2_cmds.c/.h`, `XO2_dev.c/.h`)

Upstream: Lattice Semiconductor reference code (c) 2011-2012, no version
identifier (SBOM `openwater:version-status: undetermined`). Pre-existing
adaptation: `XO2_cmds.c` transport rewritten for the STM32 HAL I2C with the
TCA9548A mux select (`MachXO_CmdXfer`).

### 2026-10-22 - programming entry points compiled only into Factory builds

Reason: PTR-2026-1-5 follow-up (host-writable safety state), CFW-09. The
laser-safety FPGAs must not be erasable or programmable from a Release image;
the code must be absent, not switched off at runtime.

Change: the following functions are wrapped in
`#if defined(OW_FACTORY_PROGRAMMING)` (set only by the CMake preset
`Factory`), in both the definition and the prototype:

| File | Functions |
|---|---|
| `XO2_api.c/.h` | `XO2ECA_apiProgram`, `XO2ECA_apiClearXO2`, `XO2ECA_apiEraseFlash`, `XO2ECA_apiWriteUFM` |
| `XO2_cmds.c/.h` | `XO2ECAcmd_setUserCode`, `XO2ECAcmd_EraseFlash`, `XO2ECAcmd_SRAMErase`, `XO2ECAcmd_setDone`, `XO2ECAcmd_CfgErase`, `XO2ECAcmd_CfgWritePage`, `XO2ECAcmd_UFMErase`, `XO2ECAcmd_UFMWritePage`, `XO2ECAcmd_FeatureRowErase`, `XO2ECAcmd_FeatureRowWrite` |

Unchanged and present in every build (used by `fpga_attest.c`):
`XO2ECAcmd_readDevID`, `readUserCode`, `readTraceID`, `Bypass`, `openCfgIF`
(transparent mode), `closeCfgIF`, `readStatusReg`, `readBusyFlag`,
`waitStatusBusy`, `waitBusyFlag`, `SetPage`, `Refresh`, `CfgResetAddr`,
`CfgReadPage`, `UFMResetAddr`, `UFMReadPage`, `FeatureRowRead`,
`XO2ECA_apiReadBackCfg` (upstream stub), `XO2ECA_apiReadBackUFM`,
`XO2ECA_apiGetHdwInfo`, `XO2ECA_apiGetHdwStatus`, `XO2ECA_apiJEDECinfo`,
`XO2ECA_apiJEDECverify`.

No function body was modified. CI (`build-firmware.yml`, `clinical-build.yml`)
runs `scripts/check_release_symbols.sh` (`arm-none-eabi-nm`) on the Release
ELF and fails if any programming symbol is present.

## jsmn (`jsmn.c/.h`)

See `scripts/sbom-manifest.json` (`openwater:local-changes`): reserved
argument on `jsmn_init`/`jsmn_parse`, and the `toksuper` guard of upstream
824d9a7 (CVA O9/R7).
