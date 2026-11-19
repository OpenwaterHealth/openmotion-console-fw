#!/usr/bin/env python3
"""verify_system_odometer.py - Bench-verify the console system odometer.

The system odometer (OW_CTRL_GET_SYSTEM_ODO) reports total powered-on
minutes: the firmware accumulates HAL tick time in the main loop and
persists every 2 minutes; reads return persisted total + live elapsed, so
the value should advance by exactly 1 per 60 s of wall time.

Phases:
  1. Baseline: connect, read both odometers.
  2. Rate: stay connected and idle for --duration seconds (default 180),
     sampling every 15 s; the system odometer must advance by
     ~duration/60 (+/-1 for minute-boundary alignment). The laser
     odometer must NOT move while idle-connected (no scan runs).
  3. Port cycle: close and reopen the VCP; the system odometer must not
     jump by more than the wall time of the cycle (<=1 min).

Usage:
    python scripts/verify_system_odometer.py
    python scripts/verify_system_odometer.py --duration 300
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

from omotion import MotionInterface

CONNECT_TIMEOUT_S = 12.0
SAMPLE_PERIOD_S = 15.0


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


def read_both(iface: MotionInterface) -> tuple[int, int]:
    system_min = iface.console.get_system_odometer_minutes()
    laser = iface.console.get_laser_odometer_pulses()
    if system_min is None or laser is None:
        raise RuntimeError("firmware NAK'd an odometer read")
    return system_min, laser


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=180.0,
                    help="Idle observation window in seconds (default 180)")
    args = ap.parse_args()

    watchdog_s = args.duration + 120
    wd = threading.Timer(watchdog_s, lambda: (
        print(f"\nWATCHDOG: still running after {watchdog_s:.0f}s - aborting",
              flush=True), os._exit(2)))
    wd.daemon = True
    wd.start()

    failures: list[str] = []

    # ── Phase 1: baseline ────────────────────────────────────────────────
    print("Phase 1: connect + baseline read")
    iface = connect()
    sys0, laser0 = read_both(iface)
    t0 = time.monotonic()
    print(f"  [baseline] system = {sys0} min, laser = {laser0} pulses")

    # ── Phase 2: idle rate check ─────────────────────────────────────────
    expected_min = args.duration / 60.0
    print(f"Phase 2: idle-connected for {args.duration:.0f}s "
          f"(expect system +{expected_min:.1f} min, laser +0)")
    while time.monotonic() - t0 < args.duration:
        time.sleep(min(SAMPLE_PERIOD_S, args.duration - (time.monotonic() - t0)))
        s, l = read_both(iface)
        print(f"  [t+{time.monotonic() - t0:5.0f}s] system = {s} min "
              f"(+{s - sys0}), laser = {l} (+{l - laser0})")

    sys1, laser1 = read_both(iface)
    elapsed_min = (time.monotonic() - t0) / 60.0
    sys_delta = sys1 - sys0
    if abs(sys_delta - elapsed_min) <= 1.0:
        print(f"  PASS: system advanced +{sys_delta} min in {elapsed_min:.2f} min")
    else:
        failures.append(f"system advanced +{sys_delta} min in {elapsed_min:.2f} min "
                        "of wall time (expected match +/-1)")
        print(f"  FAIL: system advanced +{sys_delta} min in {elapsed_min:.2f} min")
    if laser1 == laser0:
        print("  PASS: laser odometer unchanged while idle-connected")
    else:
        failures.append(f"laser odometer moved +{laser1 - laser0} while "
                        "idle-connected with no scan")
        print(f"  FAIL: laser odometer moved +{laser1 - laser0} while idle")

    # ── Phase 3: port close/reopen must not jump the system odometer ────
    print("Phase 3: port close/reopen (system may gain at most the wall time)")
    t_cycle = time.monotonic()
    iface.stop()
    time.sleep(1.0)
    iface = connect()
    sys2, laser2 = read_both(iface)
    cycle_min = (time.monotonic() - t_cycle) / 60.0
    print(f"  [after reopen] system = {sys2} min (+{sys2 - sys1}), "
          f"laser = {laser2} (+{laser2 - laser1})  [cycle took {cycle_min:.2f} min]")
    if 0 <= sys2 - sys1 <= 1:
        print(f"  PASS: system delta +{sys2 - sys1} across port cycle")
    else:
        failures.append(f"system odometer jumped +{sys2 - sys1} across a "
                        f"{cycle_min:.2f} min port cycle")
        print(f"  FAIL: system delta +{sys2 - sys1} across port cycle")
    if laser2 != laser1:
        # Known issue #50: phantom laser pulses on port close. Report as info,
        # not a failure of THIS test - the laser close-path bug is tracked.
        print(f"  NOTE: laser odometer gained +{laser2 - laser1} across the "
              "port cycle (known issue #50)")

    iface.stop()

    print()
    if failures:
        print("RESULT: FAIL")
        for f in failures:
            print(f"  - {f}")
        return 1
    print(f"RESULT: PASS - system odometer advanced +{sys_delta} min over "
          f"{elapsed_min:.2f} min and behaved across the port cycle")
    return 0


if __name__ == "__main__":
    sys.exit(main())
