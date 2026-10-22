#!/usr/bin/env python3
"""
safety_ufm_record.py - build or decode the per-unit safety record of a
safety FPGA UFM (Core/Inc/safety_ufm_record.h, layout 1).

The record carries the per-unit limits WI-00015 safety calibration derives
for one safety device (EE or OPT). Its layout is the one the safety FPGA's
own loader reads (openmotion-safety-fpga src/ufm_config.v,
tools/make_ufm_record.py): UFM pages 0-1, magic "OW", raw register values,
16-bit truncated byte-sum checksum. It is written into the UFM of that
device by a FACTORY firmware image through the OW_FPGA_PROG UFM page
commands (SDK MotionConsole.fpga_prog_ufm_*); a Release image reads and
validates it at boot (fpga_attest.c) and has no code that writes it.

Values are given in engineering units and converted to raw register values
rounding towards the conservative side (upper limits down, the rate lower
limit up), so a record built from in-envelope values is always inside the
envelope. The envelope below mirrors Core/Inc/safety_envelope.h; a record
outside it latches the firmware interlock (TRIG_INTERLOCK_FPGA_ATTEST), so
this tool refuses to build one.

Usage:
  python scripts/safety_ufm_record.py build --device EE --drive-cl 4287 \\
      --pulse-ul 550 [--pulse-ll 0 --rate-ll 23125 --pwm 140 --cw 140] -o ee.bin [--mem ee.mem]
  python scripts/safety_ufm_record.py decode --device EE ee.bin
  python scripts/safety_ufm_record.py decode --device OPT --hex 4f5701...

--mem writes the Diamond .mem rows (byte 15 leftmost) for bench programming
with the Diamond Programmer "FLASH UFM Erase" / "FLASH UFM Program,Verify"
operations (openmotion-safety-fpga docs/ufm_calibration.md).

Standard library only.
"""
from __future__ import annotations

import argparse
import math
import struct
import sys

MAGIC = b"OW"
LAYOUT = 1
SIZE = 32
PAGE = 0
PAGE_BYTES = 16
DEVICES = ("EE", "OPT")
# magic, layout, reserved, pw_ll, pw_ul, rate_ll, drive_cl, pwm, cw, reserved[8], checksum
FMT = "<2sBBIIIHHH8sH"

# Register LSBs (Core/Src/fpga_register_map.c, safety_envelope.h SAFETY_FPGA_*_LSB)
TIME_LSB_US = 0.320
DRIVE_LSB = 1.86
CURRENT_LSB_MA = 0.160

# Envelope, engineering units: Core/Inc/safety_envelope.h
PULSE_UL_MAX_US = 660
RATE_LL_MIN_US = 23125
DRIVE_CL_MAX = {"EE": 5000, "OPT": 7200}
PWM_MAX_MA = 140
CW_MAX_MA = 140


def checksum(body: bytes) -> int:
    return sum(body[:30]) & 0xFFFF


def floor_raw(eng: float, lsb: float) -> int:
    return math.floor(eng / lsb + 1e-9)


def ceil_raw(eng: float, lsb: float) -> int:
    return math.ceil(eng / lsb - 1e-9)


def envelope_problems(device: str, raw: dict) -> list[str]:
    """The firmware's safety_ufm_limits_in_envelope(), on raw values."""
    p = []
    if raw["pw_ll"] > raw["pw_ul"]:
        p.append("pulse_ll above pulse_ul")
    if raw["pw_ul"] * TIME_LSB_US > PULSE_UL_MAX_US:
        p.append(f"pulse_ul {raw['pw_ul'] * TIME_LSB_US:.2f} us > {PULSE_UL_MAX_US}")
    if raw["rate_ll"] * TIME_LSB_US < RATE_LL_MIN_US:
        p.append(f"rate_ll {raw['rate_ll'] * TIME_LSB_US:.2f} us < {RATE_LL_MIN_US}")
    if raw["drive_cl"] == 0 or raw["drive_cl"] * DRIVE_LSB > DRIVE_CL_MAX[device]:
        p.append(f"drive_cl {raw['drive_cl'] * DRIVE_LSB:.2f} outside (0, {DRIVE_CL_MAX[device]}]")
    if raw["pwm"] * CURRENT_LSB_MA > PWM_MAX_MA:
        p.append(f"pwm {raw['pwm'] * CURRENT_LSB_MA:.2f} mA > {PWM_MAX_MA}")
    if raw["cw"] * CURRENT_LSB_MA > CW_MAX_MA:
        p.append(f"cw {raw['cw'] * CURRENT_LSB_MA:.2f} mA > {CW_MAX_MA}")
    return p


def build(device: str, pulse_ll: float, pulse_ul: float, rate_ll: float, drive_cl: float,
          pwm: float, cw: float) -> bytes:
    raw = {
        "pw_ll": floor_raw(pulse_ll, TIME_LSB_US),
        "pw_ul": floor_raw(pulse_ul, TIME_LSB_US),
        "rate_ll": ceil_raw(rate_ll, TIME_LSB_US),
        "drive_cl": floor_raw(drive_cl, DRIVE_LSB),
        "pwm": floor_raw(pwm, CURRENT_LSB_MA),
        "cw": floor_raw(cw, CURRENT_LSB_MA),
    }
    problems = envelope_problems(device, raw)
    if problems:
        raise SystemExit("outside the envelope (safety_envelope.h): " + "; ".join(problems))
    body = struct.pack(FMT[:-1], MAGIC, LAYOUT, 0, raw["pw_ll"], raw["pw_ul"], raw["rate_ll"],
                       raw["drive_cl"], raw["pwm"], raw["cw"], bytes(8))
    assert len(body) == SIZE - 2
    return body + struct.pack("<H", checksum(body))


def decode(device: str, rec: bytes) -> dict:
    if len(rec) < SIZE:
        raise SystemExit(f"record is {len(rec)} bytes, need {SIZE}")
    rec = rec[:SIZE]
    if rec == bytes(SIZE) or rec == b"\xff" * SIZE:
        return {"status": "blank (interlock)"}
    magic, layout, res0, pw_ll, pw_ul, rate_ll, drive_cl, pwm, cw, res, csum = struct.unpack(FMT, rec)
    raw = dict(pw_ll=pw_ll, pw_ul=pw_ul, rate_ll=rate_ll, drive_cl=drive_cl, pwm=pwm, cw=cw)
    out = dict(magic=magic.decode("latin1"), layout=layout,
               pulse_width_ll=f"0x{pw_ll:08x} = {pw_ll * TIME_LSB_US:.2f} us",
               pulse_width_ul=f"0x{pw_ul:08x} = {pw_ul * TIME_LSB_US:.2f} us",
               rate_ll=f"0x{rate_ll:08x} = {rate_ll * TIME_LSB_US:.2f} us",
               drive_cl=f"0x{drive_cl:04x} = {drive_cl * DRIVE_LSB:.2f}",
               pwm_current=f"0x{pwm:04x} = {pwm * CURRENT_LSB_MA:.2f} mA",
               cw_current=f"0x{cw:04x} = {cw * CURRENT_LSB_MA:.2f} mA",
               checksum=f"0x{csum:04x}")
    if magic != MAGIC:
        out["status"] = "bad magic"
    elif checksum(rec) != csum:
        out["status"] = "bad checksum"
    elif layout != LAYOUT:
        out["status"] = "unknown layout"
    elif res0 != 0 or res != bytes(8):
        out["status"] = "reserved bytes set"
    else:
        problems = envelope_problems(device, raw)
        out["status"] = "valid" if not problems else "outside envelope: " + "; ".join(problems)
    return out


def to_mem(rec: bytes) -> list[str]:
    """Diamond .mem rows, one UFM page per row, byte 15 leftmost."""
    return ["".join(f"{b:02X}" for b in reversed(rec[i:i + PAGE_BYTES])) for i in range(0, len(rec), PAGE_BYTES)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="build a record")
    b.add_argument("--device", choices=DEVICES, required=True)
    b.add_argument("--pulse-ul", type=float, required=True, help="EE/OPT_PULSE_WIDTH_UL, us (TA pulse width x 1.1)")
    b.add_argument("--pulse-ll", type=float, default=0.0, help="EE/OPT_PULSE_WIDTH_LL, us (design reset 0)")
    b.add_argument("--rate-ll", type=float, default=float(RATE_LL_MIN_US), help="EE/OPT_RATE_LL, us")
    b.add_argument("--drive-cl", type=float, required=True, help="EE/OPT_DRIVE_CL (register-map units, x 1.86 per LSB)")
    b.add_argument("--pwm", type=float, default=140.0, help="PWM current limit, mA (design reset 140)")
    b.add_argument("--cw", type=float, default=140.0, help="CW current limit, mA (design reset 140)")
    b.add_argument("-o", "--output", help="write the 32 bytes here (two 16-byte UFM pages)")
    b.add_argument("--mem", help="write Diamond .mem rows here")
    d = sub.add_parser("decode", help="decode a record")
    d.add_argument("--device", choices=DEVICES, required=True)
    d.add_argument("file", nargs="?")
    d.add_argument("--hex", help="record as hex text instead of a file")
    a = ap.parse_args()

    if a.cmd == "build":
        rec = build(a.device, a.pulse_ll, a.pulse_ul, a.rate_ll, a.drive_cl, a.pwm, a.cw)
        print(f"UFM page {PAGE}: {rec[:16].hex()}")
        print(f"UFM page {PAGE + 1}: {rec[16:].hex()}")
        if a.output:
            with open(a.output, "wb") as f:
                f.write(rec)
            print(f"wrote {a.output}")
        if a.mem:
            with open(a.mem, "w", encoding="ascii") as f:
                f.write("\n".join(to_mem(rec)) + "\n")
            print(f"wrote {a.mem}")
        return 0
    if a.hex:
        rec = bytes.fromhex(a.hex)
    else:
        with open(a.file, "rb") as f:
            rec = f.read()
    for k, v in decode(a.device, rec).items():
        print(f"{k:>16}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
