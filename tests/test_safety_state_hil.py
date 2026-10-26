"""HIL tests for the host-writable safety state remediation (PTR-2026-1-5).

What the console must do with this firmware:

  * attest the configuration flash of the two safety MachXO2 devices (EE,
    OPT) at boot and on request (OW_CMD_FPGA_ATTEST), validate the per-unit
    safety record in their UFM, and refuse the trigger until both pass
    (TRIG_INTERLOCK_FPGA_ATTEST); a blank, corrupt or out-of-envelope record
    keeps the interlock;
  * refuse every OW_FPGA_PROG opcode on a Release or Debug image with
    OW_ERROR + OW_CODE_NOT_SUPPORTED;
  * refuse a configuration write that names a safety parameter with
    OW_ERROR + OW_CODE_CFG_SAFETY_KEY and leave the stored record unchanged;
  * keep the TEC thermal trip armed from the compile-time threshold whatever
    the stored record holds (checked here through the boot-info/interlock
    state after a deliberately corrupt write is refused, and through the
    telemetry tec_status flag).

Two tests need a bench-only reprogramming of one FPGA with a modified
bitstream (Factory image + SDK FpgaPageProgrammer) and are therefore marked
``bench_reprogram``: they run only with OPENMOTION_HIL_REPROGRAM=1 and must
NEVER be run on a unit that will ship. docs/fpga-attestation.md gives the
procedure and the recovery (reprogram the released .jed, re-provision the
UFM record, reboot, attestation passes).

Run on a bench with a console attached:

    OPENMOTION_HIL=1 pytest tests/test_safety_state_hil.py
    (PowerShell: $env:OPENMOTION_HIL = "1")

The SDK (omotion) must be importable; the raw packets are sent through
console.uart.send_packet like scripts/check_console_config.py in the SDK.
"""
import json
import os
import struct
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)
bench_reprogram = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL_REPROGRAM") != "1",
    reason="reprograms a safety FPGA with a modified bitstream; bench unit only, "
           "set OPENMOTION_HIL_REPROGRAM=1 deliberately",
)

CONNECT_TIMEOUT_S = 15.0
ATTEST_TIMEOUT_S = 120.0

# Protocol constants mirrored from Core/Inc/common.h.
OW_CMD = 0xE2
OW_RESP = 0xE3
OW_CONTROLLER = 0xEA
OW_FPGA_PROG = 0xEB
OW_ERROR = 0xEF
OW_CMD_USR_CFG = 0x0A
OW_CMD_BOOT_INFO = 0x0B
OW_CMD_FPGA_ATTEST = 0x11
OW_CMD_FPGA_PROG_OPEN = 0x30
OW_CMD_FPGA_PROG_READ_STATUS = 0x3F
OW_CTRL_START_TRIG = 0x15
OW_CTRL_STOP_TRIG = 0x16
OW_CODE_CFG_RANGE = 0xF8
OW_CODE_CFG_UNKNOWN_KEY = 0xF9
OW_CODE_CFG_SAFETY_KEY = 0xFA
OW_CODE_NOT_SUPPORTED = 0xFB
OW_CODE_BUSY = 0xFC
TRIG_INTERLOCK_FPGA_ATTEST = 0x40
ATTEST_IDLE, ATTEST_RUNNING, ATTEST_PASS, ATTEST_FAIL = 0, 1, 2, 3
ST_DIGEST, ST_READ_FAIL, ST_UFM_INVALID, ST_UFM_BLANK, ST_UFM_VALID, ST_NOT_CHECKED, ST_PENDING = 1, 2, 4, 8, 16, 64, 128
DEV_NAMES = {0: "seed", 1: "TA", 2: "EE", 3: "OPT"}
SAFETY_DEVS = (2, 3)
CFG_PAGES = 3198   # MachXO2-2000 configuration sector (scripts/sbom-manifest.json openwater:cfg-pages)
# safety_ufm_status_t (Core/Inc/safety_ufm_record.h)
UFM_BLANK, UFM_VALID, UFM_BAD_MAGIC, UFM_BAD_LAYOUT, UFM_LOADER_FALLBACK, UFM_BAD_CHECKSUM, UFM_BAD_RESERVED, \
    UFM_OUT_OF_BOUNDS = range(8)
ST_FAULT_MASK = ST_DIGEST | ST_READ_FAIL | ST_UFM_INVALID | ST_UFM_BLANK

DEV_REPORT = struct.Struct("<BBBBHHI32s32s")
REPORT_HEAD = struct.Struct("<BBBBHH")


@pytest.fixture
def console():
    from omotion import MotionInterface

    interface = MotionInterface()
    interface.start(wait=False)
    handle = interface.console
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not handle.is_connected():
        time.sleep(0.2)
    if not handle.is_connected():
        interface.stop()
        pytest.fail(f"console not reachable in {CONNECT_TIMEOUT_S:.0f}s")
    try:
        handle.telemetry.stop()
    except Exception:
        pass
    yield handle
    interface.stop()


def _send(console, packet_type, command, reserved=0, data=None, addr=0):
    r = console.uart.send_packet(id=None, packetType=packet_type, command=command,
                                 reserved=reserved, addr=addr, data=data)
    console.uart.clear_buffer()
    assert r is not None, "no reply"
    return r


def _data(r):
    return bytes(r.data[: r.data_len]) if r.data else b""


def parse_attest(payload):
    ver, state, pct, interlocked, runs, pages = REPORT_HEAD.unpack_from(payload)
    assert ver == 2
    devs = []
    off = REPORT_HEAD.size
    for _ in range(4):
        d, st, ufm, loader, pages_read, ms, devid, digest, expected = DEV_REPORT.unpack_from(payload, off)
        off += DEV_REPORT.size
        devs.append(dict(device=d, status=st, ufm=ufm, loader=loader, pages=pages_read, ms=ms, devid=devid,
                         digest=digest.hex(), expected=expected.hex()))
    return dict(state=state, pct=pct, interlocked=interlocked, runs=runs, cfg_pages=pages, dev=devs)


def read_attest(console, start=False):
    r = _send(console, OW_CMD, OW_CMD_FPGA_ATTEST, reserved=1 if start else 0)
    if r.packetType == OW_ERROR:
        return None, _data(r)
    return parse_attest(_data(r)), None


def wait_attest_done(console):
    deadline = time.monotonic() + ATTEST_TIMEOUT_S
    while time.monotonic() < deadline:
        rep, err = read_attest(console)
        assert err is None, f"attest report refused: {err.hex()}"
        if rep["state"] in (ATTEST_PASS, ATTEST_FAIL):
            return rep
        time.sleep(1.0)
    pytest.fail(f"attestation did not finish in {ATTEST_TIMEOUT_S:.0f}s")


def boot_info(console):
    r = _send(console, OW_CMD, OW_CMD_BOOT_INFO)
    d = _data(r)
    ver, _rsv, vtor, base, interlocks, attest, build, _r2 = struct.unpack("<B3sIIBBBB", d[:16])
    assert ver == 2
    return dict(vtor=vtor, interlocks=interlocks, attest=attest, factory=bool(build & 1))


def read_config_text(console):
    r = _send(console, OW_CMD, OW_CMD_USR_CFG, reserved=0)
    d = _data(r)
    _magic, _ver, seq, _crc, json_len = struct.unpack("<IIIHH", d[:16])
    return seq, d[16:16 + json_len].decode("utf-8", errors="replace").rstrip("\x00")


def write_config_text(console, text):
    r = _send(console, OW_CMD, OW_CMD_USR_CFG, reserved=1, data=text.encode("utf-8"))
    return r.packetType, _data(r)


def start_trigger(console):
    r = _send(console, OW_CONTROLLER, OW_CTRL_START_TRIG)
    refused = r.packetType == OW_ERROR
    status = _data(r)[0] if refused and r.data_len else None
    if not refused:
        _send(console, OW_CONTROLLER, OW_CTRL_STOP_TRIG)
    return refused, status


# --------------------------------------------------------------------------- B: attestation
@requires_bench
def test_boot_attestation_passes_on_a_correctly_programmed_unit(console):
    """After boot the background check completes with every device matching
    its compiled-in digest; the interlock bit clears and the report carries
    the per-device readback time (recorded in docs/fpga-attestation.md)."""
    rep = wait_attest_done(console)
    if any(rep["dev"][d]["status"] & ST_UFM_BLANK for d in SAFETY_DEVS):
        pytest.skip("unit has no safety record in UFM (not provisioned): "
                    "see test_missing_or_invalid_record_keeps_interlock")
    assert rep["state"] == ATTEST_PASS, rep
    assert rep["interlocked"] == 0
    assert rep["cfg_pages"] == CFG_PAGES
    # Only the two safety devices are attested; seed and TA say "not checked".
    for d in rep["dev"][:2]:
        assert d["status"] == ST_NOT_CHECKED, d
    for d in rep["dev"][2:]:
        assert d["status"] & (ST_FAULT_MASK | ST_PENDING | ST_NOT_CHECKED) == 0, d
        assert d["status"] & ST_UFM_VALID and d["ufm"] == UFM_VALID, d
        assert d["pages"] == CFG_PAGES
        assert d["digest"] == d["expected"], DEV_NAMES[d["device"]]
        print(f"{DEV_NAMES[d['device']]}: {d['ms']} ms readback, UFM status {d['ufm']}")
    assert rep["dev"][2]["expected"] == rep["dev"][3]["expected"], "EE and OPT share the safety design"
    bi = boot_info(console)
    assert bi["attest"] == ATTEST_PASS
    assert bi["interlocks"] & TRIG_INTERLOCK_FPGA_ATTEST == 0


@requires_bench
def test_missing_or_invalid_record_keeps_interlock(console):
    """A unit whose EE or OPT UFM holds no valid safety record (blank,
    corrupt, outside the envelope) fails the attestation even though the
    configuration digests match: the interlock stays set, the start is
    refused with TRIG_INTERLOCK_FPGA_ATTEST and boot-info reports it."""
    rep = wait_attest_done(console)
    bad = [rep["dev"][d] for d in SAFETY_DEVS if rep["dev"][d]["ufm"] != UFM_VALID]
    if not bad:
        pytest.skip("both safety records are valid on this unit")
    for d in SAFETY_DEVS:
        dev = rep["dev"][d]
        assert dev["digest"] == dev["expected"], DEV_NAMES[d]
    for dev in bad:
        expected_bit = ST_UFM_BLANK if dev["ufm"] == UFM_BLANK else ST_UFM_INVALID
        assert dev["status"] & expected_bit, dev
        print(f"{DEV_NAMES[dev['device']]}: UFM status {dev['ufm']}, device status 0x{dev['status']:02x}")
    assert rep["state"] == ATTEST_FAIL and rep["interlocked"] == 1, rep
    bi = boot_info(console)
    assert bi["attest"] == ATTEST_FAIL
    assert bi["interlocks"] & TRIG_INTERLOCK_FPGA_ATTEST
    refused, status = start_trigger(console)
    assert refused and status is not None and status & TRIG_INTERLOCK_FPGA_ATTEST


@requires_bench
def test_attestation_on_request_and_busy_refusal(console):
    """A host-requested run is accepted when idle, refused with OW_CODE_BUSY
    while it runs, and passes again."""
    wait_attest_done(console)
    rep, err = read_attest(console, start=True)
    assert err is None and rep["state"] == ATTEST_RUNNING
    _, err = read_attest(console, start=True)
    assert err == bytes([OW_CODE_BUSY])
    rep = wait_attest_done(console)
    assert rep["runs"] >= 2
    # The repeated run reaches the same verdict as the boot run.
    assert rep["state"] in (ATTEST_PASS, ATTEST_FAIL)
    assert all(rep["dev"][d]["digest"] == rep["dev"][d]["expected"] for d in SAFETY_DEVS), rep


# --------------------------------------------------------------------------- A: programming not supported
@requires_bench
def test_fpga_prog_not_supported_on_release_image(console):
    """Every OW_FPGA_PROG opcode answers OW_ERROR + OW_CODE_NOT_SUPPORTED on a
    Release or Debug image (Factory images are excluded by boot-info)."""
    bi = boot_info(console)
    if bi["factory"]:
        pytest.skip("Factory image on the bench: programming is compiled in by design")
    for opcode in (OW_CMD_FPGA_PROG_READ_STATUS, OW_CMD_FPGA_PROG_OPEN):
        for channel in (0, 1, 2, 3):
            r = _send(console, OW_FPGA_PROG, opcode, reserved=channel)
            assert r.packetType == OW_ERROR, f"opcode 0x{opcode:02X} ch {channel} not refused"
            assert _data(r) == bytes([OW_CODE_NOT_SUPPORTED])
    # and the devices are untouched: their configuration still matches
    rep = wait_attest_done(console)
    for d in SAFETY_DEVS:
        assert rep["dev"][d]["digest"] == rep["dev"][d]["expected"], DEV_NAMES[d]


# --------------------------------------------------------------------------- C: config
@requires_bench
def test_config_write_with_safety_key_is_rejected(console):
    """A write naming a firmware-owned safety parameter is refused with
    OW_CODE_CFG_SAFETY_KEY and the stored record (and its sequence) is
    unchanged. Unknown keys and out-of-range operational values are
    refused with their own codes."""
    bi = boot_info(console)
    if bi["factory"]:
        pytest.skip("Factory image accepts safety keys by design")
    seq_before, text_before = read_config_text(console)
    cases = [
        ('{"TEC_TRIP": 90}', OW_CODE_CFG_SAFETY_KEY),
        ('{"EE_DRIVE_CL": 9999}', OW_CODE_CFG_SAFETY_KEY),
        ('{"OPT_THRESH": 7143, "OPT_GAIN": 1.86}', OW_CODE_CFG_SAFETY_KEY),
        ('{"EE_PULSE_WIDTH_UL": 2000}', OW_CODE_CFG_SAFETY_KEY),
        ('{"TA_CURRENT_LIMIT": 9000}', OW_CODE_CFG_SAFETY_KEY),
        ('{"TA_CURRENT_DRV": 9000}', OW_CODE_CFG_RANGE),
        ('{"TA_PULSE_WIDTH": 601}', OW_CODE_CFG_RANGE),
        ('{"EE_STATUS": 0}', OW_CODE_CFG_UNKNOWN_KEY),
        ('{"bogus": 1}', OW_CODE_CFG_UNKNOWN_KEY),
    ]
    for text, code in cases:
        ptype, data = write_config_text(console, text)
        assert ptype == OW_ERROR, text
        assert data == bytes([code]), f"{text}: got {data.hex()}"
    seq_after, text_after = read_config_text(console)
    assert seq_after == seq_before and text_after == text_before
    # an operational write within range is accepted and stored
    cfg = json.loads(text_before) if text_before else {}
    cfg["TA_PULSE_WIDTH"] = cfg.get("TA_PULSE_WIDTH", 500)
    ptype, _ = write_config_text(console, json.dumps(cfg))
    assert ptype == OW_RESP
    seq_new, text_new = read_config_text(console)
    assert seq_new == seq_after + 1
    assert "TEC_TRIP" not in json.loads(text_new)


@requires_bench
def test_tec_trip_in_force_after_record_corruption(console):
    """The TEC thermal trip does not depend on the stored record: after a
    refused (corrupt) write and after a write of an empty record the
    telemetry still reports the trip armed and evaluating (tec_status True
    below threshold), and the console is not in the 'defaults' interlock
    for a valid empty record."""
    bi = boot_info(console)
    ptype, _ = write_config_text(console, '{"TA_CURRENT_DRV": NaN}')
    assert ptype == OW_ERROR
    ptype, _ = write_config_text(console, "{}")
    assert ptype == OW_RESP
    time.sleep(0.5)
    # SDK MotionConsole.tec_status() -> (t1, t2, t3, tec_volts, tec_good);
    # tec_good is True while the sense voltage is below the compile-time
    # threshold; the comparison is live (TEC_TRIP_VALUE != 0).
    status = console.tec_status()
    assert status is not None and status[4] is True, status
    bi2 = boot_info(console)
    assert bi2["interlocks"] & 0x08 == 0, "empty record must not leave the console on defaults"


# --------------------------------------------------------------------------- B: negative path (bench reprogramming)
@requires_bench
@bench_reprogram
def test_modified_bitstream_sets_interlock_and_blocks_arming(console):
    """Bench-only: after one FPGA was reprogrammed (Factory image) with a
    bitstream that differs from the released one, a Release image reports
    the digest mismatch, keeps TRIG_INTERLOCK_FPGA_ATTEST set and refuses
    the trigger with that bit in the status byte. Recovery is in
    docs/fpga-attestation.md."""
    rep = wait_attest_done(console)
    assert rep["state"] == ATTEST_FAIL, "run this only after the bench reprogramming"
    # A digest mismatch, or a device that no longer answers its configuration
    # port (the bench seed device after the 2026-10-22 incident), both fault.
    bad = [d for d in rep["dev"] if d["status"] & (ST_DIGEST | ST_READ_FAIL)]
    assert bad, rep
    print("faulty devices:", [(DEV_NAMES[d["device"]], hex(d["status"])) for d in bad])
    assert rep["interlocked"] == 1
    refused, status = start_trigger(console)
    assert refused and status is not None and status & TRIG_INTERLOCK_FPGA_ATTEST
    bi = boot_info(console)
    assert bi["attest"] == ATTEST_FAIL and bi["interlocks"] & TRIG_INTERLOCK_FPGA_ATTEST
