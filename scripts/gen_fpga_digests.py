#!/usr/bin/env python3
"""
gen_fpga_digests.py - expected FPGA configuration digests for fpga_attest.c

The console firmware verifies at boot (and on OW_CMD_FPGA_ATTEST) that the
configuration flash of each MachXO2 FPGA still holds the released bitstream:
it reads the configuration pages back over the device's I2C configuration
port, hashes them with SHA-256 and compares the digest with a value compiled
into the signed firmware image. This script produces that header.

Inputs
------
scripts/sbom-manifest.json lists the released FPGA designs as components with
``openwater:role = fpga-bitstream``.  Each one records the repository, the
release tag, the JEDEC artefact file name, the SHA-256 of that artefact, the
number of configuration pages the firmware hashes and the expected digest.
The artefacts themselves are committed under::

    fpga-artefacts/<repository>/<tag>/<file>.jed

exactly as published on the GitHub release of that tag.  ``--fetch`` downloads
a missing artefact with ``gh release download``; the download is accepted only
when its SHA-256 matches the manifest.

The build FAILS when a component's tag or artefact is missing, when an artefact
does not match its recorded SHA-256, or (unless ``--update-manifest``) when the
digest computed from the artefact differs from the digest recorded in the
manifest.  Bumping an FPGA release is therefore a reviewed manifest change plus
the new artefact, never a silent rebuild.

How the digest is formed
------------------------
A MachXO2 JEDEC file lists the fuse map of the whole on-chip flash as
``L<offset>`` fields.  The host-side programmer (openmotion-sdk
``omotion/jedecParser.py`` + ``FPGAProgrammer.py``) packs those fuses into
16-byte pages, MSB-first within each byte, and writes them to the
configuration sector page by page from page 0.  The firmware reads the pages
back the same way (``XO2ECAcmd_CfgResetAddr`` then ``XO2ECAcmd_CfgReadPage``),
so the expected digest is::

    SHA-256( page[0] || page[1] || ... || page[cfg_pages - 1] )

over the packed JEDEC rows.  ``cfg_pages`` (manifest openwater:cfg-pages) is
the configuration-sector size of the device: 3198 pages for the MachXO2-2000.
The vendored Lattice ECA table ``XO2DevList`` says 3200, but a sequential
configuration read continues into the UFM after page 3197. On the bench
(2026-10-22) pages 3198 and 3199 of a 3200-page read returned UFM pages 0 and
1, i.e. the per-unit safety record. Rows beyond cfg_pages are therefore NOT
hashed: the UFM carries the per-unit safety record, which the firmware
validates on its own (see docs/fpga-attestation.md).  The script reports whether those
trailing rows are blank so a design that starts to use UFM initialisation is
noticed.

USERCODE (``U`` field), the feature row (``E`` field) and the trace ID are not
part of the configuration pages and are therefore not part of the digest.

Standard library only; runs in the build image and on a developer machine.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROLE = "fpga-bitstream"
PAGE_BYTES = 16
PAGE_BITS = PAGE_BYTES * 8
PROP_REPO = "openwater:repository"
PROP_TAG = "openwater:release-tag"
PROP_FILE = "openwater:artefact"
PROP_FILE_SHA = "openwater:artefact-sha256"
PROP_PAGES = "openwater:cfg-pages"
PROP_DIGEST = "openwater:cfg-digest-sha256"
PROP_ROLE = "openwater:role"
PROP_DEVICE = "openwater:console-device"
# "true" when the released bitstream contains the safety FPGA's UFM loader
# (ufm_config.v, status register 0x31); the firmware then also requires the
# loader to report the record applied. Required on every bitstream component.
PROP_UFM_LOADER = "openwater:ufm-loader"


class DigestError(RuntimeError):
    pass


# --------------------------------------------------------------------------- JEDEC
def parse_jedec(path: Path) -> tuple[int, bytearray, dict]:
    """Return (total_fuses, packed rows, extras) for a Diamond JEDEC file.

    Mirrors openmotion-sdk omotion/jedecParser.py: the file is split into
    '*'-terminated fields; QF gives the fuse count; every L field places its
    '0'/'1' characters from its start index; unlisted fuses are 0.  Rows are
    packed MSB-first (fuse N -> bit 7-(N%8) of byte N//8).
    """
    text = path.read_text(encoding="latin1").lstrip("\x02")
    fields = text.split("*")
    total: int | None = None
    bits: bytearray | None = None  # one byte per fuse, 0/1
    extras: dict = {}
    for raw in fields:
        field = raw.strip()
        if not field or field.upper().startswith("NOTE"):
            continue
        if field.startswith("QF"):
            num = field[2:].strip()
            if not num.isdigit():
                raise DigestError(f"{path}: bad QF field {field!r}")
            total = int(num)
            bits = bytearray(total)
        elif field.startswith("L"):
            if bits is None:
                raise DigestError(f"{path}: L field before QF")
            m = re.match(r"L(\d+)", field)
            if not m:
                continue
            pos = int(m.group(1))
            for ch in field[m.end():]:
                if ch == "0" or ch == "1":
                    if pos >= len(bits):
                        raise DigestError(f"{path}: L field exceeds QF={total}")
                    bits[pos] = 1 if ch == "1" else 0
                    pos += 1
        elif field.startswith("E"):
            lines = [ln.strip() for ln in field.splitlines() if ln.strip()]
            if lines and len(lines[0]) >= 65:
                extras["feature_row"] = lines[0][1:65]
            if len(lines) >= 2:
                extras["feabits"] = lines[1][:16]
        elif field.startswith("U"):
            extras["usercode"] = field[1:].strip()
        elif field.startswith("C"):
            extras["fuse_checksum"] = field[1:].strip()
    if total is None or bits is None or total <= 0:
        raise DigestError(f"{path}: QF field not found")

    rows = (total + PAGE_BITS - 1) // PAGE_BITS
    out = bytearray(rows * PAGE_BYTES)
    for i in range(total):
        if bits[i]:
            out[i >> 3] |= 0x80 >> (i & 7)
    return total, out, extras


def cfg_digest(packed: bytearray, cfg_pages: int) -> tuple[str, int, bool]:
    """SHA-256 over the first cfg_pages pages; also how many rows the image
    has and whether the rows beyond cfg_pages are all zero."""
    rows = len(packed) // PAGE_BYTES
    if rows < cfg_pages:
        raise DigestError(f"image has {rows} rows, fewer than cfg_pages={cfg_pages}")
    digest = hashlib.sha256(bytes(packed[: cfg_pages * PAGE_BYTES])).hexdigest()
    trailing_blank = not any(packed[cfg_pages * PAGE_BYTES:])
    return digest, rows, trailing_blank


# --------------------------------------------------------------------------- manifest
def props(component: dict) -> dict:
    return {p["name"]: p["value"] for p in component.get("properties", [])}


def set_prop(component: dict, name: str, value: str) -> None:
    for p in component.setdefault("properties", []):
        if p["name"] == name:
            p["value"] = value
            return
    component["properties"].append({"name": name, "value": value})


def bitstream_components(manifest: dict) -> list[dict]:
    comps = [c for c in manifest.get("components", []) if props(c).get(PROP_ROLE) == ROLE]
    if not comps:
        raise DigestError("manifest lists no component with openwater:role = fpga-bitstream")
    return comps


def artefact_path(root: Path, p: dict) -> Path:
    return root / p[PROP_REPO] / p[PROP_TAG] / p[PROP_FILE]


def fetch(root: Path, p: dict) -> Path:
    dest = artefact_path(root, p)
    if shutil.which("gh") is None:
        raise DigestError("--fetch needs the GitHub CLI (gh) on PATH")
    dest.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["gh", "release", "download", p[PROP_TAG], "-R", f"OpenwaterHealth/{p[PROP_REPO]}",
           "-p", p[PROP_FILE], "-D", str(dest.parent), "--clobber"]
    print("[gen_fpga_digests] " + " ".join(cmd))
    r = subprocess.run(cmd, text=True, capture_output=True)
    if r.returncode != 0:
        raise DigestError(f"download failed for {p[PROP_REPO]} {p[PROP_TAG]} {p[PROP_FILE]}: {r.stderr.strip()}")
    return dest


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


# --------------------------------------------------------------------------- header
def c_ident(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9]", "_", name).upper()


def write_header(out: Path, entries: list[dict]) -> None:
    lines = [
        "/* Generated by scripts/gen_fpga_digests.py from scripts/sbom-manifest.json",
        " * and the committed FPGA release artefacts. DO NOT EDIT. */",
        "#ifndef FPGA_ATTEST_EXPECTED_H",
        "#define FPGA_ATTEST_EXPECTED_H",
        "",
        "#include <stdint.h>",
        "",
        "/* Configuration pages hashed per device (MachXO2-2000, Lattice XO2DevList). */",
        f"#define FPGA_ATTEST_CFG_PAGES {entries[0]['cfg_pages']}U",
        f"#define FPGA_ATTEST_PAGE_BYTES {PAGE_BYTES}U",
        "",
    ]
    for e in entries:
        ident = c_ident(e["device"])
        lines.append(f"/* {e['repo']} {e['tag']} - {e['file']} (artefact sha256 {e['file_sha'][:16]}...) */")
        lines.append(f"#define FPGA_ATTEST_TAG_{ident} \"{e['tag']}\"")
        lines.append(f"#define FPGA_ATTEST_REPO_{ident} \"{e['repo']}\"")
        hexs = ", ".join(f"0x{e['digest'][i:i + 2]}" for i in range(0, 64, 2))
        lines.append(f"#define FPGA_ATTEST_DIGEST_{ident} {{ {hexs} }}")
        lines.append(f"#define FPGA_ATTEST_UFM_LOADER_{ident} {1 if e['ufm_loader'] else 0}")
        lines.append("")
    lines += ["#endif /* FPGA_ATTEST_EXPECTED_H */", ""]
    text = "\n".join(lines)
    if out.exists() and out.read_text(encoding="utf-8") == text:
        return  # keep the timestamp: no needless rebuild
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")


# --------------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--manifest", type=Path, required=True, help="scripts/sbom-manifest.json")
    ap.add_argument("--artefacts", type=Path, required=True, help="fpga-artefacts/ directory")
    ap.add_argument("-o", "--output", type=Path, help="header to write (fpga_attest_expected.h)")
    ap.add_argument("--fetch", action="store_true", help="download a missing artefact from the GitHub release")
    ap.add_argument("--update-manifest", action="store_true",
                    help="write the computed artefact SHA-256 and digest back into the manifest (reviewed change)")
    a = ap.parse_args()

    manifest = json.loads(a.manifest.read_text(encoding="utf-8"))
    entries: list[dict] = []
    failures: list[str] = []
    for c in bitstream_components(manifest):
        p = props(c)
        for key in (PROP_REPO, PROP_TAG, PROP_FILE, PROP_PAGES, PROP_DEVICE, PROP_UFM_LOADER):
            if key not in p:
                failures.append(f"{c.get('name')}: manifest property {key} missing")
        if failures:
            continue
        if p[PROP_UFM_LOADER] not in ("true", "false"):
            failures.append(f"{c['name']}: {PROP_UFM_LOADER} must be \"true\" or \"false\", got {p[PROP_UFM_LOADER]!r}")
            continue
        if c.get("version") != p[PROP_TAG]:
            failures.append(f"{c['name']}: component version {c.get('version')!r} != {PROP_TAG} {p[PROP_TAG]!r}")
            continue
        path = artefact_path(a.artefacts, p)
        if not path.is_file() and a.fetch:
            try:
                path = fetch(a.artefacts, p)
            except DigestError as e:
                failures.append(str(e))
                continue
        if not path.is_file():
            failures.append(f"{c['name']}: artefact missing: {path} (tag {p[PROP_TAG]}; run with --fetch or add the file)")
            continue
        file_sha = sha256_file(path)
        if a.update_manifest:
            set_prop(c, PROP_FILE_SHA, file_sha)
        elif p.get(PROP_FILE_SHA) != file_sha:
            failures.append(f"{c['name']}: {path.name} sha256 {file_sha} != manifest {p.get(PROP_FILE_SHA)}")
            continue
        try:
            total, packed, extras = parse_jedec(path)
            digest, rows, trailing_blank = cfg_digest(packed, int(p[PROP_PAGES]))
        except DigestError as e:
            failures.append(f"{c['name']}: {e}")
            continue
        if a.update_manifest:
            set_prop(c, PROP_DIGEST, digest)
        elif p.get(PROP_DIGEST) != digest:
            failures.append(f"{c['name']}: computed cfg digest {digest} != manifest {p.get(PROP_DIGEST)}")
            continue
        print(f"[gen_fpga_digests] {c['name']} {p[PROP_TAG]} {path.name}: QF={total} rows={rows} "
              f"hashed={p[PROP_PAGES]} pages, trailing rows {'blank' if trailing_blank else 'NOT blank'}, "
              f"usercode={extras.get('usercode')}, digest {digest[:16]}...")
        if not trailing_blank:
            print(f"[gen_fpga_digests] WARNING: {c['name']}: fuse rows beyond the configuration sector are not "
                  f"blank; the design initialises the UFM. Those rows are not attested.")
        entries.append({"device": p[PROP_DEVICE], "repo": p[PROP_REPO], "tag": p[PROP_TAG], "file": p[PROP_FILE],
                        "file_sha": file_sha, "cfg_pages": int(p[PROP_PAGES]), "digest": digest,
                        "ufm_loader": p[PROP_UFM_LOADER] == "true"})

    if failures:
        for f in failures:
            print(f"[gen_fpga_digests] ERROR: {f}", file=sys.stderr)
        return 1
    pages = {e["cfg_pages"] for e in entries}
    if len(pages) != 1:
        print(f"[gen_fpga_digests] ERROR: every bitstream must hash the same page count, got {sorted(pages)}",
              file=sys.stderr)
        return 1
    if a.update_manifest:
        a.manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(f"[gen_fpga_digests] manifest updated: {a.manifest}")
    if a.output:
        write_header(a.output, entries)
        print(f"[gen_fpga_digests] wrote {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
