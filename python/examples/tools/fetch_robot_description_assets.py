#!/usr/bin/env python3
"""Fetch local-only real robot description assets (URDF + meshes) for Robot Description demos.

Downloads pinned URDF/xacro sources for a heterogeneous showroom fleet and, unless
``--skip-meshes`` is given,
the ``package://`` meshes they reference into a self-contained ``meshes_root`` tree::

    <output-dir>/<robot>.urdf
    <output-dir>/meshes_root/<package>/<relative-path-to-mesh>

The mesh tree lets the HORUS SDK resolve ``package://`` meshes without installing any
ROS description packages, by pointing ``mesh_root`` at ``meshes_root`` (the resolver
still prefers the ament index when the packages *are* installed).
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import urllib.request
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Set, Tuple


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
SPOT_DESCRIPTION_COMMIT = "156d1802bfb117f219dbfce7597d283d5fabc968"
SPOT_SIMPLE_XACRO_URL = (
    "https://raw.githubusercontent.com/bdaiinstitute/spot_description/"
    f"{SPOT_DESCRIPTION_COMMIT}/spot_description/urdf/spot_simple.urdf.xacro"
)
UNITREE_SHOWROOM_COMMIT = "d6f13aad60320ce1d60a07b82a76b5a553f2a0a9"

UNITREE_SHOWROOM_URDFS: Dict[str, Tuple[str, str]] = {
    "a1": ("a1.urdf", "robots/a1_description/urdf/a1.urdf"),
    "a2": ("a2.urdf", "robots/a2_description/urdf/a2.urdf"),
    "go2": ("go2.urdf", "robots/go2_description/urdf/go2_description.urdf"),
    "b2": ("b2.urdf", "robots/b2_description/urdf/b2_description.urdf"),
    "b2w": ("b2w.urdf", "robots/b2w_description/urdf/b2w_description.urdf"),
    "g1": ("g1.urdf", "robots/g1_description/g1_29dof.urdf"),
}

# Raw URL base for each description package such that a `package://<pkg>/<rel>` URI maps
# to `<base>/<rel>`. Pinned to the same commits as the URDF sources above.
MESH_PACKAGE_BASES: Dict[str, str] = {
    "jackal_description": (
        "https://raw.githubusercontent.com/jackal/jackal/"
        "9978ac0c7ebf9d730879aca4d9ade1c67ff1f3b1/jackal_description"
    ),
    "anymal_c_simple_description": (
        "https://raw.githubusercontent.com/ANYbotics/anymal_c_simple_description/"
        "f67f50e152ae7a1d381fc4a3ee279edbc9b21984"
    ),
    "go1_description": (
        "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        "4590b76ec8fb2412cdbe21c82044a07131e181e3/robots/go1_description"
    ),
    "h1_description": (
        "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        "34e7506c5333666f7c6dad6bfeeca2176cace70b/robots/h1_description"
    ),
    "a1_description": (
        f"https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        f"{UNITREE_SHOWROOM_COMMIT}/robots/a1_description"
    ),
    "a2_description": (
        f"https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        f"{UNITREE_SHOWROOM_COMMIT}/robots/a2_description"
    ),
    "go2_description": (
        f"https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        f"{UNITREE_SHOWROOM_COMMIT}/robots/go2_description"
    ),
    "b2_description": (
        f"https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        f"{UNITREE_SHOWROOM_COMMIT}/robots/b2_description"
    ),
    "b2w_description": (
        f"https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        f"{UNITREE_SHOWROOM_COMMIT}/robots/b2w_description"
    ),
    "g1_description": (
        f"https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        f"{UNITREE_SHOWROOM_COMMIT}/robots/g1_description"
    ),
    "spot_description": (
        "https://raw.githubusercontent.com/bdaiinstitute/spot_description/"
        f"{SPOT_DESCRIPTION_COMMIT}/spot_description"
    ),
}

_PACKAGE_MESH_RE = re.compile(r'filename\s*=\s*"(package://[^"]+)"')
_TEXTURE_SUFFIXES = {".bmp", ".gif", ".jpeg", ".jpg", ".png", ".tga", ".tif", ".tiff"}


def _download_text(url: str) -> str:
    with urllib.request.urlopen(url, timeout=30) as response:
        return response.read().decode("utf-8", errors="replace")


def _download_binary(url: str, timeout: float = 60.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read()


def _write_text(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(payload, encoding="utf-8")


def _sanitize_jackal_xacro(payload: str) -> str:
    lines: List[str] = []
    for line in payload.splitlines():
        stripped = line.strip()
        if "<xacro:include" in stripped:
            # Keep the demo self-contained: strip external includes and extras hooks.
            continue
        lines.append(line)
    return "\n".join(lines) + "\n"


def _sanitize_static_xacro_payload(payload: str) -> str:
    """Convert xacro-free XML that only declares the xacro namespace into plain URDF text."""
    return payload.replace(' xmlns:xacro="http://www.ros.org/wiki/xacro"', "")


def _normalize_relative_mesh_refs(payload: str, package_name: str) -> str:
    """Normalize URDF-local mesh references into package:// references.

    Some upstream Unitree descriptions use paths like ``meshes/foo.STL`` or
    ``../meshes/foo.STL``. The HORUS demos store meshes under ``meshes_root`` by
    package name, so normalizing these references keeps resolution deterministic.
    """

    if not package_name:
        return payload

    def repl(match: re.Match) -> str:
        quote = match.group(1)
        raw_ref = match.group(2).strip().replace("\\", "/")
        if (
            raw_ref.startswith("package://")
            or raw_ref.startswith("file://")
            or raw_ref.startswith("/")
            or "://" in raw_ref
        ):
            return match.group(0)

        while raw_ref.startswith("../"):
            raw_ref = raw_ref[3:]
        if raw_ref.startswith("./"):
            raw_ref = raw_ref[2:]
        if not raw_ref.startswith("meshes/"):
            return match.group(0)

        return f"filename={quote}package://{package_name}/{raw_ref}{quote}"

    return re.sub(r'filename\s*=\s*([\'"])([^\'"]+)\1', repl, payload)


def _try_expand_xacro(xacro_path: Path) -> Tuple[str, str]:
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


def _collect_package_meshes(urdf_text: str) -> Set[Tuple[str, str]]:
    """Return the set of unique (package, relative_path) mesh references in a URDF."""
    refs: Set[Tuple[str, str]] = set()
    for match in _PACKAGE_MESH_RE.finditer(urdf_text):
        uri = match.group(1)
        remainder = uri[len("package://"):]
        package_name, _, relative_path = remainder.partition("/")
        if package_name and relative_path:
            refs.add((package_name, relative_path))
    return refs


def _download_meshes(
    urdf_files: List[Path],
    mesh_root: Path,
    force: bool,
) -> Dict[str, int]:
    refs: Set[Tuple[str, str]] = set()
    for urdf_file in urdf_files:
        if not urdf_file.is_file():
            continue
        try:
            refs |= _collect_package_meshes(urdf_file.read_text(encoding="utf-8", errors="replace"))
        except Exception as exc:  # pragma: no cover - defensive
            print(f"[robot-description-assets] warning: could not scan {urdf_file}: {exc}")

    stats = {"downloaded": 0, "skipped": 0, "failed": 0, "total": len(refs)}
    missing_bases: Set[str] = set()
    for package_name, relative_path in sorted(refs):
        base = MESH_PACKAGE_BASES.get(package_name)
        if not base:
            missing_bases.add(package_name)
            stats["failed"] += 1
            continue

        target = mesh_root / package_name / relative_path
        if target.is_file() and not force:
            stats["skipped"] += 1
            continue

        url = f"{base}/{relative_path}"
        try:
            data = _download_binary(url)
        except Exception as exc:
            print(f"[robot-description-assets]   mesh FAIL {package_name}/{relative_path}: {exc}")
            stats["failed"] += 1
            continue

        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        stats["downloaded"] += 1

    if missing_bases:
        print(
            "[robot-description-assets] warning: no download base for packages: "
            + ", ".join(sorted(missing_bases))
        )
    return stats


def _dae_texture_refs(dae_path: Path) -> Set[str]:
    try:
        root = ET.parse(dae_path).getroot()
    except Exception:
        return set()

    namespace = ""
    if root.tag.startswith("{") and "}" in root.tag:
        namespace = root.tag[1:].split("}", 1)[0]

    def tag(name: str) -> str:
        return f"{{{namespace}}}{name}" if namespace else name

    refs: Set[str] = set()
    for image_el in root.findall(f".//{tag('image')}"):
        init_from_el = image_el.find(tag("init_from"))
        raw_ref = str(init_from_el.text or "").strip() if init_from_el is not None else ""
        if not raw_ref:
            continue
        if raw_ref.startswith("file://"):
            raw_ref = raw_ref[7:]
        suffix = Path(raw_ref).suffix.lower()
        if suffix in _TEXTURE_SUFFIXES:
            refs.add(raw_ref.replace("\\", "/"))
    return refs


def _download_dae_textures(mesh_root: Path, force: bool) -> Dict[str, int]:
    dae_files = sorted(mesh_root.glob("**/*.dae"))
    stats = {"downloaded": 0, "skipped": 0, "failed": 0, "total": 0}
    for dae_path in dae_files:
        try:
            relative_to_root = dae_path.relative_to(mesh_root)
        except ValueError:
            continue
        parts = relative_to_root.parts
        if len(parts) < 2:
            continue
        package_name = parts[0]
        base = MESH_PACKAGE_BASES.get(package_name)
        if not base:
            continue
        dae_relative_parent = Path(*parts[1:]).parent
        for texture_ref in sorted(_dae_texture_refs(dae_path)):
            stats["total"] += 1
            texture_path = Path(texture_ref)
            if texture_path.is_absolute():
                print(f"[robot-description-assets]   texture SKIP absolute {texture_ref}")
                stats["failed"] += 1
                continue
            target = dae_path.parent / texture_path
            if target.is_file() and not force:
                stats["skipped"] += 1
                continue
            url_path = (dae_relative_parent / texture_path).as_posix()
            url = f"{base}/{url_path}"
            try:
                data = _download_binary(url)
            except Exception as exc:
                print(f"[robot-description-assets]   texture FAIL {package_name}/{url_path}: {exc}")
                stats["failed"] += 1
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            stats["downloaded"] += 1
    return stats


def build_parser() -> argparse.ArgumentParser:
    script_dir = Path(__file__).resolve().parent
    default_output = script_dir.parent / ".local_assets" / "robot_descriptions"
    parser = argparse.ArgumentParser(
        description=(
            "Download pinned robot-description sources for the HORUS 11-robot showroom fleet "
            "and their package:// meshes into local-only demo assets. Expands the Jackal "
            "xacro to URDF when xacro tooling is available."
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
        help="Overwrite existing files (URDFs and meshes) in the output folder.",
    )
    parser.add_argument(
        "--skip-meshes",
        action="store_true",
        help="Only fetch URDF/xacro text (use when ROS description packages are installed).",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    mesh_root = output_dir / "meshes_root"

    go1_path = output_dir / "go1.urdf"
    anymal_c_path = output_dir / "anymal_c.urdf"
    jackal_xacro_path = output_dir / "jackal.urdf.xacro"
    jackal_urdf_path = output_dir / "jackal.urdf"
    h1_path = output_dir / "h1.urdf"
    spot_xacro_path = output_dir / "spot_simple.urdf.xacro"
    spot_urdf_path = output_dir / "spot.urdf"
    showroom_paths: Dict[str, Path] = {
        name: output_dir / filename
        for name, (filename, _source_path) in UNITREE_SHOWROOM_URDFS.items()
    }
    sources_path = output_dir / "SOURCES.json"

    # --- URDF/xacro sources (per-file skip unless --force) -------------------------
    url_by_path = {
        go1_path: GO1_URDF_URL,
        anymal_c_path: ANYMAL_C_URDF_URL,
        h1_path: H1_URDF_URL,
        spot_xacro_path: SPOT_SIMPLE_XACRO_URL,
    }
    for path, url in url_by_path.items():
        if path.exists() and not args.force:
            print(f"[robot-description-assets] present {path}")
            continue
        _write_text(path, _download_text(url))
        print(f"[robot-description-assets] wrote {path}")

    for robot_name, (filename, source_path) in UNITREE_SHOWROOM_URDFS.items():
        path = showroom_paths[robot_name]
        if path.exists() and not args.force:
            print(f"[robot-description-assets] present {path}")
            continue

        package_name = source_path.split("/", 2)[1]
        url = f"https://raw.githubusercontent.com/unitreerobotics/unitree_ros/{UNITREE_SHOWROOM_COMMIT}/{source_path}"
        payload = _normalize_relative_mesh_refs(_download_text(url), package_name)
        _write_text(path, payload)
        print(f"[robot-description-assets] wrote {path}")

    if jackal_xacro_path.exists() and not args.force:
        print(f"[robot-description-assets] present {jackal_xacro_path}")
    else:
        _write_text(jackal_xacro_path, _sanitize_jackal_xacro(_download_text(JACKAL_XACRO_URL)))
        print(f"[robot-description-assets] wrote {jackal_xacro_path}")

    if not jackal_urdf_path.exists() or args.force:
        expanded_urdf, xacro_error = _try_expand_xacro(jackal_xacro_path)
        if expanded_urdf:
            _write_text(jackal_urdf_path, expanded_urdf)
            print(f"[robot-description-assets] wrote {jackal_urdf_path} (expanded from xacro)")
        else:
            print("[robot-description-assets] warning: Jackal xacro could not be expanded.")
            print(f"[robot-description-assets] warning detail: {xacro_error}")
            print("[robot-description-assets] use jackal.urdf.xacro with resolver xacro support.")

    if not spot_urdf_path.exists() or args.force:
        expanded_urdf, xacro_error = _try_expand_xacro(spot_xacro_path)
        if expanded_urdf:
            _write_text(spot_urdf_path, expanded_urdf)
            print(f"[robot-description-assets] wrote {spot_urdf_path} (expanded from xacro)")
        else:
            fallback_urdf = _sanitize_static_xacro_payload(spot_xacro_path.read_text(encoding="utf-8"))
            _write_text(spot_urdf_path, fallback_urdf)
            print(f"[robot-description-assets] wrote {spot_urdf_path} (static xacro fallback)")
            print(f"[robot-description-assets] note: xacro expansion unavailable: {xacro_error}")

    # --- meshes --------------------------------------------------------------------
    mesh_stats = {"downloaded": 0, "skipped": 0, "failed": 0, "total": 0}
    if args.skip_meshes:
        print("[robot-description-assets] --skip-meshes set; not fetching meshes.")
    else:
        scan_urdfs = [go1_path, anymal_c_path, h1_path]
        scan_urdfs.extend(showroom_paths.values())
        scan_urdfs.append(jackal_urdf_path if jackal_urdf_path.exists() else jackal_xacro_path)
        scan_urdfs.append(spot_urdf_path if spot_urdf_path.exists() else spot_xacro_path)
        print(f"[robot-description-assets] fetching meshes into {mesh_root} ...")
        mesh_stats = _download_meshes(scan_urdfs, mesh_root, args.force)
        texture_stats = _download_dae_textures(mesh_root, args.force)
        print(
            "[robot-description-assets] meshes: "
            f"{mesh_stats['downloaded']} downloaded, {mesh_stats['skipped']} present, "
            f"{mesh_stats['failed']} failed (of {mesh_stats['total']} referenced)."
        )
        print(
            "[robot-description-assets] textures: "
            f"{texture_stats['downloaded']} downloaded, {texture_stats['skipped']} present, "
            f"{texture_stats['failed']} failed (of {texture_stats['total']} referenced)."
        )
    if args.skip_meshes:
        texture_stats = {"downloaded": 0, "skipped": 0, "failed": 0, "total": 0}

    # --- provenance manifest -------------------------------------------------------
    sources_manifest = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "mesh_root": str(mesh_root),
        "mesh_stats": mesh_stats,
        "texture_stats": texture_stats,
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
                "notes": "Includes sanitized for self-contained demo use.",
            },
            {
                "robot": "h1",
                "type": "urdf",
                "path": str(h1_path),
                "source_url": H1_URDF_URL,
                "license_hint": "See upstream unitreerobotics/unitree_ros LICENSE.",
            },
            {
                "robot": "spot",
                "type": "xacro",
                "path": str(spot_xacro_path),
                "source_url": SPOT_SIMPLE_XACRO_URL,
                "license_hint": "See upstream bdaiinstitute/spot_description LICENSE.",
            },
        ],
    }
    for robot_name, (filename, source_path) in UNITREE_SHOWROOM_URDFS.items():
        sources_manifest["assets"].append(
            {
                "robot": robot_name,
                "type": "urdf",
                "path": str(showroom_paths[robot_name]),
                "source_url": (
                    f"https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
                    f"{UNITREE_SHOWROOM_COMMIT}/{source_path}"
                ),
                "license_hint": "See upstream unitreerobotics/unitree_ros LICENSE.",
                "notes": "Relative mesh references normalized to package:// for local HORUS showroom use.",
            }
        )
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
    if spot_urdf_path.exists():
        sources_manifest["assets"].append(
            {
                "robot": "spot",
                "type": "urdf",
                "path": str(spot_urdf_path),
                "source": str(spot_xacro_path),
                "generated_by": "xacro expansion",
            }
        )

    _write_text(sources_path, json.dumps(sources_manifest, indent=2, sort_keys=True))
    print(f"[robot-description-assets] wrote {sources_path}")
    print("[robot-description-assets] done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
