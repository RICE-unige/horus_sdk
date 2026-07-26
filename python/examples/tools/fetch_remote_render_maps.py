#!/usr/bin/env python3
"""Download the five official ETH3D clean scans used by HORUS remote rendering."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys

from horus.remote_rendering import (
    DEFAULT_ETH3D_REMOTE_ROOT,
    ETH3D_REMOTE_SCENES,
    ETH3D_REMOTE_SCENE_IDS,
    prepare_eth3d_remote_scene,
    prepare_eth3d_remote_scene_lod,
)


ETH3D_DATASET_PAGE = "https://www.eth3d.net/datasets"
ETH3D_LICENSE = "CC BY-NC-SA 4.0"
ETH3D_CITATION = (
    "T. Schops et al., A Multi-View Stereo Benchmark with High-Resolution "
    "Images and Multi-Camera Videos, CVPR 2017."
)


def _ensure_py7zr(root: Path) -> Path:
    environment = root / "archive-tools"
    python = environment / "bin" / "python"
    if not python.is_file():
        subprocess.run([sys.executable, "-m", "venv", str(environment)], check=True)
    probe = subprocess.run(
        [str(python), "-c", "import py7zr"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if probe.returncode != 0:
        subprocess.run(
            [
                str(python),
                "-m",
                "pip",
                "install",
                "--disable-pip-version-check",
                "py7zr==1.1.3",
            ],
            check=True,
        )
    return python


def _find_scene_root(extraction_root: Path) -> Path | None:
    if not extraction_root.is_dir():
        return None
    matches = sorted(extraction_root.rglob("scan_alignment.mlp"))
    return matches[0].parent if matches else None


def _download(url: str, destination: Path, force: bool) -> None:
    if destination.is_file() and destination.stat().st_size > 100_000_000 and not force:
        print(f"[ok]  {destination} ({destination.stat().st_size / 1024**2:.1f} MiB)")
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_suffix(destination.suffix + ".part")
    print(f"[get] {url}")
    subprocess.run(
        [
            "curl",
            "--fail",
            "--location",
            "--retry",
            "5",
            "--retry-all-errors",
            "--continue-at",
            "-",
            "--output",
            str(partial),
            url,
        ],
        check=True,
    )
    partial.replace(destination)


def _fetch_scene(scene_id: str, root: Path, force: bool, python: Path) -> dict:
    config = ETH3D_REMOTE_SCENES[scene_id]
    scene_root = root / scene_id
    archive = scene_root / f"{scene_id}_scan_clean.7z"
    extraction_root = scene_root / "scan_clean"
    existing = _find_scene_root(extraction_root)
    if existing is None or force:
        if force and extraction_root.exists():
            shutil.rmtree(extraction_root)
        _download(str(config["archive_url"]), archive, force)
        extraction_root.mkdir(parents=True, exist_ok=True)
        print(f"[extract] {archive}")
        subprocess.run(
            [
                str(python),
                "-c",
                (
                    "import py7zr,sys; "
                    "py7zr.SevenZipFile(sys.argv[1]).extractall(sys.argv[2])"
                ),
                str(archive),
                str(extraction_root),
            ],
            check=True,
        )
        existing = _find_scene_root(extraction_root)
    if existing is None:
        raise RuntimeError(f"{scene_id} extracted without scan_alignment.mlp")
    print(f"[ready] {scene_id}: {existing}")
    return {
        "scene_id": scene_id,
        "label": config["label"],
        "source_url": config["archive_url"],
        "dataset_page": ETH3D_DATASET_PAGE,
        "archive": str(archive),
        "archive_bytes": archive.stat().st_size,
        "scan_root": str(existing),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        action="append",
        choices=ETH3D_REMOTE_SCENE_IDS,
        help="Scene to fetch. Repeat the option; omit it to fetch all five.",
    )
    parser.add_argument("--root", type=Path, default=DEFAULT_ETH3D_REMOTE_ROOT)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--source-voxel-size", type=float, default=0.0125)
    parser.add_argument("--no-prepare-lod", action="store_true")
    args = parser.parse_args()
    if not 0.001 <= args.source_voxel_size <= 0.25:
        parser.error("--source-voxel-size must be between 0.001 and 0.25 metres")

    root = args.root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    python = _ensure_py7zr(root)
    scene_ids = tuple(args.scene or ETH3D_REMOTE_SCENE_IDS)
    records = [
        _fetch_scene(scene_id, root, args.force, python)
        for scene_id in scene_ids
    ]
    if not args.no_prepare_lod:
        for record in records:
            scene = prepare_eth3d_remote_scene(
                record["scene_id"],
                record["scan_root"],
            )
            lod_path, lod_point_count = prepare_eth3d_remote_scene_lod(
                scene,
                voxel_size_m=args.source_voxel_size,
            )
            record["source_point_count"] = scene.point_count
            record["render_lod_point_count"] = lod_point_count
            record["render_lod_voxel_size_m"] = args.source_voxel_size
            record["render_lod"] = str(lod_path)
            print(
                f"[lod] {scene.scene_id}: {scene.point_count:,} source -> "
                f"{lod_point_count:,} render points",
                flush=True,
            )
    manifest_path = root / "manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "generated_at": datetime.now(timezone.utc).isoformat(),
                "dataset": "ETH3D high-resolution multi-view clean laser scans",
                "dataset_page": ETH3D_DATASET_PAGE,
                "license": ETH3D_LICENSE,
                "citation": ETH3D_CITATION,
                "scenes": records,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"[manifest] {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
