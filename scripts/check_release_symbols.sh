#!/usr/bin/env bash
# Fail if a Release/Debug ELF contains any FPGA/UFM programming symbol.
# The programming paths exist only in Factory builds (OW_FACTORY_PROGRAMMING);
# a clinical image must not contain them (PTR-2026-1-5 follow-up, CFW-09).
#
#   scripts/check_release_symbols.sh build/Release/motion-console-fw.elf
set -euo pipefail
ELF="${1:?usage: $0 <elf>}"
NM="${NM:-arm-none-eabi-nm}"
PATTERN='XO2ECA_apiProgram|XO2ECA_apiClearXO2|apiEraseFlash|apiWriteUFM|XO2ECAcmd_EraseFlash|XO2ECAcmd_SRAMErase|XO2ECAcmd_CfgErase|XO2ECAcmd_CfgWritePage|XO2ECAcmd_UFMErase|XO2ECAcmd_UFMWritePage|FeatureRowErase|FeatureRowWrite|XO2ECAcmd_setDone|XO2ECAcmd_setUserCode|process_fpga_prog_command|apply_factory_override'
FOUND="$("$NM" "$ELF" | grep -E "$PATTERN" || true)"
if [ -n "$FOUND" ]; then
  echo "::error::programming symbols present in $ELF:" >&2
  echo "$FOUND" >&2
  exit 1
fi
echo "OK: no FPGA programming symbols in $ELF"
