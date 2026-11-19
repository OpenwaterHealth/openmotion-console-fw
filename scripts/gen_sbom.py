#!/usr/bin/env python3
"""
gen_sbom.py — build-time CycloneDX 1.6 SBOM for an Open-Motion firmware or bootloader image.

One copy of this file lives in each repository under scripts/ (console firmware, sensor
firmware, both bootloaders); this is the reference copy. Standard library only.

What is DERIVED at build time (never hand-maintained):
  - the root component version (git describe, or --version), commit, build config
  - STM32H7xx HAL, CMSIS Device and CMSIS-Core versions, read from the driver headers in the tree
  - the toolchain string (--toolchain, from arm-none-eabi-gcc --version inside the build image)
  - the build container digest (--build-image)
  - the bundled bootloader (firmware images): repository, tag, SHA-256 of the binary
  - the signing key (firmware images): KMS key version resource name and public-key fingerprint
  - the generation timestamp (SOURCE_DATE_EPOCH if set, else now)

What comes from the repository's reviewed MANIFEST (sbom-manifest.json next to this script):
  components whose versions are not machine-readable in the tree (vendor libraries without a
  version file, local snapshots such as jsmn, support-status notes, evidence strings, licences).
  The manifest is CycloneDX component objects; it is reviewed in pull requests like code.

Usage (CI):
  python3 scripts/gen_sbom.py --root . --manifest scripts/sbom-manifest.json \
      --name openmotion-console-fw --version 1.8.2-rc.0 --config Release \
      --toolchain "$(arm-none-eabi-gcc --version | head -1)" \
      --build-image ghcr.io/openwaterhealth/stm32-build-env@sha256:... \
      --bundled-bootloader open-motion-console-bl 1.2.0 <sha256> \
      --signing-key projects/.../cryptoKeyVersions/1 --signing-key-fingerprint <sha256> \
      -o _artifacts/openmotion-console-fw-1.8.2-rc.0.cdx.json
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import re
import subprocess
import sys
import uuid
from pathlib import Path

ORG = {"name": "Openwater (Open Water Internet Inc.)", "url": ["https://www.openwater.health"]}
GH = "https://github.com/OpenwaterHealth/"
TOOL_VERSION = "1.0.0"


# --------------------------------------------------------------------------- helpers
def git(root: Path, *args: str, default: str = "") -> str:
    try:
        return subprocess.check_output(["git", *args], cwd=root, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return default


def read_define(path: Path, name: str) -> str | None:
    """Return the numeric value of `#define NAME (0x..UL)` / `#define NAME (5U)` style macros."""
    if not path.is_file():
        return None
    m = re.search(r"#define\s+%s\s+\(?\s*(0x[0-9A-Fa-f]+|\d+)" % re.escape(name), path.read_text(encoding="utf-8", errors="replace"))
    return m.group(1) if m else None


def hal_version(root: Path) -> str | None:
    f = root / "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c"
    parts = [read_define(f, "__STM32H7xx_HAL_VERSION_MAIN"), read_define(f, "__STM32H7xx_HAL_VERSION_SUB1"),
             read_define(f, "__STM32H7xx_HAL_VERSION_SUB2")]
    if not all(parts):
        return None
    return ".".join(str(int(p, 0)) for p in parts)


def cmsis_device_version(root: Path) -> str | None:
    f = root / "Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h"
    for prefix in ("__STM32H7xx_CMSIS_VERSION", "__STM32H7_CMSIS_VERSION", "__CMSIS_DEVICE_VERSION"):
        parts = [read_define(f, prefix + "_MAIN"), read_define(f, prefix + "_SUB1"), read_define(f, prefix + "_SUB2")]
        if all(parts):
            return ".".join(str(int(p, 0)) for p in parts)
    return None


def cmsis_core_version(root: Path) -> str | None:
    f = root / "Drivers/CMSIS/Include/cmsis_version.h"
    main, sub = read_define(f, "__CM_CMSIS_VERSION_MAIN"), read_define(f, "__CM_CMSIS_VERSION_SUB")
    if main is None or sub is None:
        return None
    return f"{int(main, 0)}.{int(sub, 0)}.0"


def comp(name: str, version: str | None, ctype: str = "library", **kw) -> dict:
    c: dict = {"type": ctype, "name": name}
    if version:
        c["version"] = version
    c["bom-ref"] = kw.pop("bom_ref", None) or re.sub(r"[^A-Za-z0-9._-]+", "-", f"{name}@{version or 'unversioned'}").strip("-")
    props = kw.pop("properties", None)
    c.update({k: v for k, v in kw.items() if v is not None})
    if props:
        c["properties"] = [{"name": k, "value": str(v)} for k, v in props.items()]
    return c


def st_supplier() -> dict:
    return {"name": "STMicroelectronics", "url": ["https://www.st.com"]}


# --------------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--root", type=Path, default=Path("."), help="repository root")
    ap.add_argument("--manifest", type=Path, required=True, help="sbom-manifest.json (reviewed static components)")
    ap.add_argument("--name", required=True, help="root component name, e.g. openmotion-console-fw")
    ap.add_argument("--repo", help="GitHub repository name (default: --name)")
    ap.add_argument("--type", default="firmware", help="root component type (firmware)")
    ap.add_argument("--description", default="", help="root component description (default: from manifest)")
    ap.add_argument("--version", help="root version (default: git describe --tags --always)")
    ap.add_argument("--commit", help="commit SHA (default: git rev-parse HEAD)")
    ap.add_argument("--config", default="Release", help="build configuration")
    ap.add_argument("--toolchain", help="toolchain string, e.g. output of arm-none-eabi-gcc --version | head -1")
    ap.add_argument("--build-image", help="build container reference, ideally with digest")
    ap.add_argument("--bundled-bootloader", nargs=3, metavar=("REPO", "TAG", "SHA256"),
                    help="bootloader bundled into the production image")
    ap.add_argument("--signing-key", help="KMS crypto key version resource name used to sign this image")
    ap.add_argument("--signing-key-fingerprint", help="SHA-256 of the signing public key (DER SPKI)")
    ap.add_argument("--property", action="append", default=[], metavar="NAME=VALUE", help="extra root property")
    ap.add_argument("-o", "--output", type=Path, required=True)
    a = ap.parse_args()

    root = a.root.resolve()
    manifest = json.loads(a.manifest.read_text(encoding="utf-8"))
    repo = a.repo or a.name
    version = a.version or git(root, "describe", "--tags", "--always", "--dirty", default="unknown")
    commit = a.commit or git(root, "rev-parse", "HEAD", default="unknown")
    ts_epoch = os.environ.get("SOURCE_DATE_EPOCH")
    ts = (dt.datetime.fromtimestamp(int(ts_epoch), dt.timezone.utc) if ts_epoch
          else dt.datetime.now(dt.timezone.utc)).strftime("%Y-%m-%dT%H:%M:%SZ")

    # ---- root component ----------------------------------------------------------
    root_props = {
        "openwater:build-config": a.config,
        "openwater:commit": commit,
        "openwater:repository": GH + repo,
    }
    if a.build_image:
        root_props["openwater:build-image"] = a.build_image
    if a.signing_key:
        root_props["openwater:signing-key"] = a.signing_key
    if a.signing_key_fingerprint:
        root_props["openwater:signing-public-key-sha256"] = a.signing_key_fingerprint
    for kv in a.property:
        k, _, v = kv.partition("=")
        root_props[k] = v
    rootc = comp(a.name, version, a.type, bom_ref=f"pkg:github/OpenwaterHealth/{repo}@{version}",
                 supplier=ORG, description=a.description or manifest.get("description"),
                 purl=f"pkg:github/OpenwaterHealth/{repo}@{version}", properties=root_props)

    components: list[dict] = []

    # ---- derived from the tree ---------------------------------------------------
    hal = hal_version(root)
    if hal:
        components.append(comp("STM32H7xx HAL Driver", hal, supplier=st_supplier(),
                               purl=f"pkg:github/STMicroelectronics/stm32h7xx-hal-driver@v{hal}",
                               licenses=[{"license": {"id": "BSD-3-Clause"}}],
                               properties={"openwater:evidence": "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c __STM32H7xx_HAL_VERSION_*"}))
    dev = cmsis_device_version(root)
    if dev:
        components.append(comp("CMSIS Device STM32H7xx", dev, supplier=st_supplier(),
                               purl=f"pkg:github/STMicroelectronics/cmsis-device-h7@v{dev}",
                               licenses=[{"license": {"id": "Apache-2.0"}}],
                               properties={"openwater:evidence": "Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h __STM32H7xx_CMSIS_VERSION_*"}))
    core = cmsis_core_version(root)
    if core:
        components.append(comp("CMSIS-Core(M)", core, supplier={"name": "Arm Limited"},
                               purl=f"pkg:github/ARM-software/CMSIS_5@{core}",
                               licenses=[{"license": {"id": "Apache-2.0"}}],
                               properties={"openwater:evidence": "Drivers/CMSIS/Include/cmsis_version.h __CM_CMSIS_VERSION_*"}))
    if a.toolchain:
        m = re.search(r"(\d+\.\d+\.Rel\d+|\d+\.\d+\.\d+)", a.toolchain)
        components.append(comp("Arm GNU Toolchain (arm-none-eabi-gcc)", m.group(1) if m else None, "application",
                               supplier={"name": "Arm Limited"}, scope="excluded",
                               properties={"openwater:evidence": a.toolchain.strip()}))
    if a.build_image:
        img, _, digest = a.build_image.partition("@")
        components.append(comp(img, digest or "latest", "container", scope="excluded",
                               properties={"openwater:evidence": "docker pull in the release workflow"}))
    if a.bundled_bootloader:
        brepo, btag, bsha = a.bundled_bootloader
        components.append(comp(f"openmotion-bl ({brepo})", btag, "firmware", supplier=ORG,
                               bom_ref=f"pkg:github/OpenwaterHealth/{brepo}@{btag}",
                               purl=f"pkg:github/OpenwaterHealth/{brepo}@{btag}",
                               hashes=[{"alg": "SHA-256", "content": bsha}],
                               properties={"openwater:evidence": f"gs://openwater-firmware-artifacts/{brepo}/{btag}/{brepo}-{btag}.bin, SHA256SUMS",
                                           "openwater:consumers": "placed at 0x08000000 in the production image; verifies and launches this firmware"}))

    # ---- manifest (reviewed) -------------------------------------------------------
    derived_names = {c["name"] for c in components}
    for mc in manifest.get("components", []):
        if mc.get("name") in derived_names:
            continue  # the tree is the source of truth for these
        c = dict(mc)
        c.setdefault("bom-ref", re.sub(r"[^A-Za-z0-9._-]+", "-", f"{c['name']}@{c.get('version', 'unversioned')}").strip("-"))
        components.append(c)

    # ---- assemble ------------------------------------------------------------------
    bom = {
        "$schema": "http://cyclonedx.org/schema/bom-1.6.schema.json",
        "bomFormat": "CycloneDX",
        "specVersion": "1.6",
        "serialNumber": f"urn:uuid:{uuid.uuid4()}",
        "version": 1,
        "metadata": {
            "timestamp": ts,
            "lifecycles": [{"phase": "build"}],
            "tools": {"components": [comp("gen_sbom.py", TOOL_VERSION, "application", supplier=ORG,
                                          properties={"openwater:evidence": "scripts/gen_sbom.py, run by the release workflow"})]},
            "authors": [{"name": "Openwater firmware CI"}],
            "supplier": ORG,
            "component": rootc,
            "properties": [{"name": "openwater:generated-by", "value": "CI build of the tagged commit; static components from scripts/sbom-manifest.json"}],
        },
        "components": components,
        "dependencies": [{"ref": rootc["bom-ref"], "dependsOn": [c["bom-ref"] for c in components if c.get("scope") != "excluded"]}]
                        + [{"ref": c["bom-ref"], "dependsOn": []} for c in components],
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(bom, indent=2) + "\n", encoding="utf-8")
    digest = hashlib.sha256(a.output.read_bytes()).hexdigest()
    print(f"[gen_sbom] {a.output}: {a.name} {version} ({a.config}), {len(components)} components, sha256 {digest[:16]}...")
    for c in components:
        print(f"           - {c['type']:<11} {c['name']} {c.get('version', '')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
