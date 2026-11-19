#!/usr/bin/env python3
"""verify_laser_odometer.py - Bench-verify the console laser odometer.

Checks that the laser odometer (OW_CTRL_GET_LASER_ODO) advances by exactly
the number of LSYNC pulses fired during a scan, and by nothing else.

Phases:
  1. Baseline: connect, read the laser odometer and live LSYNC counter.
  2. Scan delta: run a --duration second trigger scan at 40 Hz and check
     the odometer advanced by duration*40 (+/-1 frame of start/stop jitter).
  3. Idle port-close delta: close and reopen the VCP with NO scan in
     between and check the odometer did NOT move. The firmware treats a
     host port close (DTR drop) as a Trigger_Stop, which also runs
     Odometer_Scan_Finish - this phase catches any double-count on that
     path.
  4. Repeat phase 3 once more to see whether a phantom increment repeats.

The odometer counts LASER_TIMER CC1 compare events, which fire whenever the
trigger runs regardless of the nTRIG / enSyncOUT gate GPIOs, so by default
this script runs with EnableTaTrigger and EnableSyncOut false and nothing
physically fires. Pass --fire-ta / --sync-out to drive the real outputs.

Usage:
    python scripts/verify_laser_odometer.py
    python scripts/verify_laser_odometer.py --duration 1.0 --fire-ta
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

from omotion import MotionInterface
from omotion.config import merge_trigger_config

CONNECT_TIMEOUT_S = 12.0
FRAME_RATE_HZ = 40


def connect() -> MotionInterface:
    iface = MotionInterface()
    iface.start(wait=False)
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not iface.console.is_connected():
        time.sleep(0.2)
    if not iface.console.is_connected():
        iface.stop()
        raise RuntimeError(f"console not connected within {CONNECT_TIMEOUT_S:.0f}s")
    return iface


def read_odo(iface: MotionInterface, label: str) -> int:
    pulses = iface.console.get_laser_odometer_pulses()
    if pulses is None:
        raise RuntimeError(
            "console NAK'd OW_CTRL_GET_LASER_ODO - firmware predates the odometer"
        )
    lsync = iface.console.get_lsync_pulsecount()
    print(f"  [{label}] laser odometer = {pulses}  (live lsync_counter = {lsync})")
    return pulses


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=1.0,
                    help="Scan duration in seconds (default 1.0)")
    ap.add_argument("--fire-ta", action="store_true",
                    help="Leave EnableTaTrigger on (real TA laser output)")
    ap.add_argument("--sync-out", action="store_true",
                    help="Leave EnableSyncOut on (fsync to sensor modules)")
    args = ap.parse_args()

    # Never hang the bench: kill the process if anything blocks.
    watchdog_s = args.duration + 90
    wd = threading.Timer(watchdog_s, lambda: (
        print(f"\nWATCHDOG: still running after {watchdog_s:.0f}s - aborting",
              flush=True), os._exit(2)))
    wd.daemon = True
    wd.start()

    expected = round(args.duration * FRAME_RATE_HZ)
    failures: list[str] = []

    # ── Phase 1: baseline ────────────────────────────────────────────────
    print("Phase 1: connect + baseline read")
    iface = connect()
    base = read_odo(iface, "baseline")

    # ── Phase 2: scan delta ──────────────────────────────────────────────
    print(f"Phase 2: {args.duration:.1f}s scan at {FRAME_RATE_HZ} Hz "
          f"(expect odometer +{expected})")
    cfg = merge_trigger_config({
        "EnableTaTrigger": bool(args.fire_ta),
        "EnableSyncOut": bool(args.sync_out),
    })
    if iface.console.set_trigger_json(cfg) is None:
        iface.stop()
        raise RuntimeError("set_trigger_json failed")
    # The firmware counts pulses from when it processes START to when it
    # processes STOP, so command latency legitimately widens the window
    # beyond the host-side sleep. Bracket the true window from the host:
    # it is at least (start-ack .. pre-stop) and at most (pre-start ..
    # stop-ack), and the count must fall inside that envelope (+/-1 pulse
    # for frame-boundary alignment).
    t_pre_start = time.monotonic()
    if not iface.console.start_trigger():
        iface.stop()
        raise RuntimeError("start_trigger failed (interlock tripped?)")
    t_post_start = time.monotonic()
    time.sleep(args.duration)
    t_pre_stop = time.monotonic()
    if not iface.console.stop_trigger():
        iface.stop()
        raise RuntimeError("stop_trigger failed")
    t_post_stop = time.monotonic()

    lo = int((t_pre_stop - t_post_start) * FRAME_RATE_HZ) - 1
    hi = int((t_post_stop - t_pre_start) * FRAME_RATE_HZ) + 1
    after_scan = read_odo(iface, "after scan")
    scan_delta = after_scan - base
    print(f"  host window: {t_pre_stop - t_post_start:.3f}s inner / "
          f"{t_post_stop - t_pre_start:.3f}s outer -> allow +{lo}..+{hi}")
    if lo <= scan_delta <= hi:
        print(f"  PASS: scan delta = +{scan_delta} (expected ~{expected}, "
              f"within measured window)")
    else:
        failures.append(f"scan delta = +{scan_delta}, outside the possible "
                        f"window +{lo}..+{hi} (expected ~{expected})")
        print(f"  FAIL: scan delta = +{scan_delta}, allowed +{lo}..+{hi}")

    # ── Phases 3-4: idle port close/reopen must not move the odometer ────
    prev = after_scan
    for phase, name in ((3, "first"), (4, "second")):
        print(f"Phase {phase}: {name} port close/reopen with no scan (expect +0)")
        iface.stop()          # closes the VCP -> firmware sees PORT_CLOSE
        time.sleep(1.0)
        iface = connect()
        now = read_odo(iface, f"after {name} reopen")
        idle_delta = now - prev
        if idle_delta == 0:
            print("  PASS: idle delta = +0")
        else:
            failures.append(
                f"{name} idle close/reopen moved the odometer by +{idle_delta} "
                "with no scan run")
            print(f"  FAIL: idle delta = +{idle_delta} (phantom pulses)")
        prev = now

    iface.stop()

    print()
    if failures:
        print("RESULT: FAIL")
        for f in failures:
            print(f"  - {f}")
        return 1
    print(f"RESULT: PASS - odometer advanced +{scan_delta} for the scan and "
          "+0 across idle port cycles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
