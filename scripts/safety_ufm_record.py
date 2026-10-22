#!/usr/bin/env python3
"""
safety_ufm_record.py - build or decode the per-unit safety record of a
safety FPGA UFM (Core/Inc/safety_ufm_record.h, layout 1).

The record carries the per-unit limits WI-00015 safety calibration derives
for one safety device (EE: device 2, OPT: device 3). It is written into UFM
pages 600-601 of that device by a FACTORY firmware image through the
OW_FPGA_PROG UFM page commands (SDK MotionConsole.fpga_prog_ufm_*); a Release
image reads and validates it at boot (fpga_attest.c). The hard bounds below
mirror Core/Inc/safety_params.h; a record outside them is refused by the
firmware (TRIG_INTERLOCK_FPGA_ATTEST), so this tool refuses to build one.

Usage:
  python scripts/safety_ufm_record.py build --device EE --drive-cl 4287 \
      --pulse-ul 550 --rate-ll 23125 [--pwm 140 --cw 140 --ta-max 4500 --ta-pulse-max 600] -o ee.bin
  python scripts/safety_ufm_record.py decode ee.bin
  python scripts/safety_ufm_record.py decode --hex 4f5753520102...

Standard library only.
"""
from __future__ import annotations

import argparse
import struct
import sys

MAGIC = b"OWSR"
LAYOUT = 1
SIZE = 32
PAGE = 600
DEVICES = {"EE": 2, "OPT": 3}
FMT = "<4sBBHIHHHHH8sH"   # magic, layout, device, pulse_ul, rate_ll, drive_cl, pwm, cw, ta_max, ta_pulse_max, reserved, crc

# Hard bounds: Core/Inc/safety_params.h
BOUNDS = {
    "pulse_ul": (0, 660),
    "rate_ll": (23125, 0xFFFFFFFF),
    "drive_cl": {"EE": (1, 5000), "OPT": (1, 7200)},
    "pwm": (0, 140),
    "cw": (0, 140),
    "ta_max": (0, 5000),
    "ta_pulse_max": (0, 600),
}


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(device: str, pulse_ul: int, rate_ll: int, drive_cl: int, pwm: int, cw: int,
          ta_max: int, ta_pulse_max: int) -> bytes:
    checks = [("pulse_ul", pulse_ul, BOUNDS["pulse_ul"]), ("rate_ll", rate_ll, BOUNDS["rate_ll"]),
              ("drive_cl", drive_cl, BOUNDS["drive_cl"][device]), ("pwm", pwm, BOUNDS["pwm"]),
              ("cw", cw, BOUNDS["cw"]), ("ta_max", ta_max, BOUNDS["ta_max"]),
              ("ta_pulse_max", ta_pulse_max, BOUNDS["ta_pulse_max"])]
    for name, value, (lo, hi) in checks:
        if not lo <= value <= hi:
            raise SystemExit(f"{name}={value} outside the hard bound [{lo}, {hi}] (safety_params.h)")
    body = struct.pack(FMT[:-1], MAGIC, LAYOUT, DEVICES[device], pulse_ul, rate_ll, drive_cl, pwm, cw,
                       ta_max, ta_pulse_max, bytes(8))
    assert len(body) == SIZE - 2
    return body + struct.pack("<H", crc16_ccitt(body))


def decode(raw: bytes) -> dict:
    if len(raw) < SIZE:
        raise SystemExit(f"record is {len(raw)} bytes, need {SIZE}")
    raw = raw[:SIZE]
    if raw == bytes(SIZE) or raw == b"\xff" * SIZE:
        return {"status": "blank"}
    magic, layout, device, pulse_ul, rate_ll, drive_cl, pwm, cw, ta_max, ta_pulse_max, reserved, crc = struct.unpack(FMT, raw)
    out = dict(magic=magic.decode("latin1"), layout=layout, device=device, pulse_width_ul_us=pulse_ul,
               rate_ll_us=rate_ll, drive_cl=drive_cl, pwm_current_ma=pwm, cw_current_ma=cw,
               ta_current_max_ma=ta_max, ta_pulse_max_us=ta_pulse_max, crc=f"0x{crc:04x}")
    if magic != MAGIC:
        out["status"] = "bad magic"
    elif layout != LAYOUT:
        out["status"] = "unknown layout"
    elif crc16_ccitt(raw[:30]) != crc:
        out["status"] = "bad checksum"
    elif reserved != bytes(8):
        out["status"] = "reserved bytes set"
    else:
        out["status"] = "valid"
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="build a record")
    b.add_argument("--device", choices=sorted(DEVICES), required=True)
    b.add_argument("--pulse-ul", type=int, required=True, help="EE/OPT_PULSE_WIDTH_UL, us (TA pulse width x 1.1)")
    b.add_argument("--rate-ll", type=int, default=23125, help="EE/OPT_RATE_LL, us")
    b.add_argument("--drive-cl", type=int, required=True, help="EE/OPT_DRIVE_CL (register-map units)")
    b.add_argument("--pwm", type=int, default=140)
    b.add_argument("--cw", type=int, default=140)
    b.add_argument("--ta-max", type=int, default=0, help="per-unit ceiling for TA_CURRENT_DRV, mA (0 = firmware bound)")
    b.add_argument("--ta-pulse-max", type=int, default=0, help="per-unit ceiling for TA_PULSE_WIDTH, us (0 = firmware bound)")
    b.add_argument("-o", "--output", help="write the 32 bytes here (two 16-byte UFM pages)")
    d = sub.add_parser("decode", help="decode a record")
    d.add_argument("file", nargs="?")
    d.add_argument("--hex", help="record as hex text instead of a file")
    a = ap.parse_args()

    if a.cmd == "build":
        rec = build(a.device, a.pulse_ul, a.rate_ll, a.drive_cl, a.pwm, a.cw, a.ta_max, a.ta_pulse_max)
        print(f"UFM page {PAGE}: {rec[:16].hex()}")
        print(f"UFM page {PAGE + 1}: {rec[16:].hex()}")
        if a.output:
            open(a.output, "wb").write(rec)
            print(f"wrote {a.output}")
        return 0
    raw = bytes.fromhex(a.hex) if a.hex else open(a.file, "rb").read()
    for k, v in decode(raw).items():
        print(f"{k:>20}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
