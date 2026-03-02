#!/usr/bin/env python3
"""Fetch local-only real robot description assets for Robot Description V1 demos."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import List


GO1_URDF_URL = (
    "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
    "4590b76ec8fb2412cdbe21c82044a07131e181e3/robots/go1_description/urdf/go1.urdf"
)
ANYMAL_C_URDF_URL = (
    "https://raw.githubusercontent.com/ANYbotics/anymal_c_simple_description/"
    "f67f50e152ae7a1d381fc4a3ee279edbc9b21984/urdf/anymal.urdf"
)
JACKAL_XACRO_URL = (
    "https://raw.githubusercontent.com/jackal/jackal/"
    "9978ac0c7ebf9d730879aca4d9ade1c67ff1f3b1/jackal_description/urdf/jackal.urdf.xacro"
)
H1_URDF_URL = (
    "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
    "34e7506c5333666f7c6dad6bfeeca2176cace70b/robots/h1_description/urdf/h1.urdf"
)


def _download_text(url: str) -> str:
    with urllib.request.urlopen(url, timeout=30) as response:
        return response.read().decode("utf-8", errors="replace")


def _write_text(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(payload, encoding="utf-8")


def _sanitize_jackal_xacro(payload: str) -> str:
    lines: List[str] = []
    for line in payload.splitlines():
        stripped = line.strip()
        if "<xacro:include" in stripped:
            # Keep Robot Description V1 self-contained: strip external includes and extras hooks.
            continue
        lines.append(line)
    return "\n".join(lines) + "\n"


def _try_expand_xacro(xacro_path: Path) -> tuple[str, str]:
    commands: List[List[str]] = []
    xacro_exec = shutil.which("xacro")
    if xacro_exec:
        commands.append([xacro_exec, str(xacro_path)])

    ros2_exec = shutil.which("ros2")
    if ros2_exec:
        commands.append([ros2_exec, "run", "xacro", "xacro", str(xacro_path)])

    commands.append(["python3", "-m", "xacro", str(xacro_path)])

    last_error = "xacro executable/module not found."
    for command in commands:
        try:
            proc = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=30,
                cwd=str(xacro_path.parent),
            )
        except Exception as exc:
            last_error = f"failed running {' '.join(command)}: {exc}"
            continue

        if proc.returncode != 0:
            stderr = str(proc.stderr or proc.stdout or "").strip()
            last_error = stderr or f"command returned {proc.returncode}: {' '.join(command)}"
            continue

        output = str(proc.stdout or "")
        if "<robot" in output:
            return output, ""
        last_error = f"xacro output missing <robot> tag for {' '.join(command)}"

    return "", last_error


def build_parser() -> argparse.ArgumentParser:
    script_dir = Path(__file__).resolve().parent
    default_output = script_dir.parent / ".local_assets" / "robot_descriptions"
    parser = argparse.ArgumentParser(
        description=(
            "Download pinned robot-description sources into local-only demo assets "
            "(Go1, Jackal, Anymal C, Unitree H1). "
            "Attempts to expand Jackal xacro to URDF when xacro tooling is available."
        )
    )
    parser.add_argument(
        "--output-dir",
        default=str(default_output),
        help="Output folder for local robot description assets.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing files in the output folder.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    go1_path = output_dir / "go1.urdf"
    anymal_c_path = output_dir / "anymal_c.urdf"
    jackal_xacro_path = output_dir / "jackal.urdf.xacro"
    jackal_urdf_path = output_dir / "jackal.urdf"
    h1_path = output_dir / "h1.urdf"
    legacy_spot_paths = [
        output_dir / "spot.urdf",
        output_dir / "spot.urdf.xacro",
    ]
    sources_path = output_dir / "SOURCES.json"

    if (
        not args.force
        and go1_path.exists()
        and anymal_c_path.exists()
        and jackal_xacro_path.exists()
        and h1_path.exists()
        and sources_path.exists()
    ):
        print(f"[robot-description-assets] already present: {output_dir}")
        return 0

    for legacy_path in legacy_spot_paths:
        if legacy_path.exists():
            legacy_path.unlink(missing_ok=True)
            print(f"[robot-description-assets] removed legacy asset {legacy_path}")

    go1_payload = _download_text(GO1_URDF_URL)
    _write_text(go1_path, go1_payload)
    print(f"[robot-description-assets] wrote {go1_path}")

    anymal_c_payload = _download_text(ANYMAL_C_URDF_URL)
    _write_text(anymal_c_path, anymal_c_payload)
    print(f"[robot-description-assets] wrote {anymal_c_path}")

    jackal_xacro_payload = _sanitize_jackal_xacro(_download_text(JACKAL_XACRO_URL))
    _write_text(jackal_xacro_path, jackal_xacro_payload)
    print(f"[robot-description-assets] wrote {jackal_xacro_path}")

    h1_payload = _download_text(H1_URDF_URL)
    _write_text(h1_path, h1_payload)
    print(f"[robot-description-assets] wrote {h1_path}")

    expanded_urdf, xacro_error = _try_expand_xacro(jackal_xacro_path)
    if expanded_urdf:
        _write_text(jackal_urdf_path, expanded_urdf)
        print(f"[robot-description-assets] wrote {jackal_urdf_path} (expanded from xacro)")
    else:
        print("[robot-description-assets] warning: Jackal xacro could not be expanded.")
        print(f"[robot-description-assets] warning detail: {xacro_error}")
        print("[robot-description-assets] use jackal.urdf.xacro with resolver xacro support or install xacro tool.")

    sources_manifest = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "assets": [
            {
                "robot": "go1",
                "type": "urdf",
                "path": str(go1_path),
                "source_url": GO1_URDF_URL,
                "license_hint": "See upstream unitreerobotics/unitree_ros LICENSE.",
            },
            {
                "robot": "anymal_c",
                "type": "urdf",
                "path": str(anymal_c_path),
                "source_url": ANYMAL_C_URDF_URL,
                "license_hint": "See upstream ANYbotics/anymal_c_simple_description LICENSE.",
            },
            {
                "robot": "jackal",
                "type": "xacro",
                "path": str(jackal_xacro_path),
                "source_url": JACKAL_XACRO_URL,
                "license_hint": "See upstream jackal/jackal LICENSE.",
                "notes": "Includes sanitized for local-only Robot Description V1 demo use.",
            },
            {
                "robot": "h1",
                "type": "urdf",
                "path": str(h1_path),
                "source_url": H1_URDF_URL,
                "license_hint": "See upstream unitreerobotics/unitree_ros LICENSE.",
            },
        ],
    }
    if jackal_urdf_path.exists():
        sources_manifest["assets"].append(
            {
                "robot": "jackal",
                "type": "urdf",
                "path": str(jackal_urdf_path),
                "source": str(jackal_xacro_path),
                "generated_by": "xacro expansion",
            }
        )

    _write_text(sources_path, json.dumps(sources_manifest, indent=2, sort_keys=True))
    print(f"[robot-description-assets] wrote {sources_path}")
    print("[robot-description-assets] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

