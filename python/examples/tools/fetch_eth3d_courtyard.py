#!/usr/bin/env python3
"""Fetch the official ETH3D Courtyard clean laser scans.

Dataset page: https://www.eth3d.net/datasets
The dataset is provided for research use under the ETH3D dataset terms.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import sys


DATASET_URL = "https://www.eth3d.net/data/courtyard_scan_clean.7z"
DEFAULT_ROOT = Path.home() / ".cache" / "horus" / "eth3d_courtyard"
EXPECTED_FILES = ("scan1.ply", "scan2.ply", "scan_alignment.mlp")


def download(url: str, destination: Path, force: bool) -> None:
    if destination.is_file() and destination.stat().st_size > 300_000_000 and not force:
        print(f"[ok]  {destination} ({destination.stat().st_size / 1024**2:.1f} MiB)")
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".part")
    temporary.unlink(missing_ok=True)
    print(f"[get] {url}")
    subprocess.run(
        [
            "curl", "--fail", "--location", "--retry", "5",
            "--retry-all-errors", "--continue-at", "-",
            "--output", str(temporary), url,
        ],
        check=True,
    )
    temporary.replace(destination)


def ensure_py7zr(root: Path) -> Path:
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
            [str(python), "-m", "pip", "install", "--disable-pip-version-check", "py7zr==1.1.3"],
            check=True,
        )
    return python


def find_dataset_root(extraction_root: Path) -> Path | None:
    for candidate in extraction_root.rglob("scan_alignment.mlp"):
        root = candidate.parent
        if all((root / filename).is_file() for filename in EXPECTED_FILES):
            return root
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    archive = root / "courtyard_scan_clean.7z"
    extraction_root = root / "scan_clean"
    existing = find_dataset_root(extraction_root) if extraction_root.exists() else None
    if existing is not None and not args.force:
        print(f"[ok]  ETH3D Courtyard is ready at {existing}")
        return 0

    if args.force and extraction_root.exists():
        shutil.rmtree(extraction_root)
    download(DATASET_URL, archive, args.force)
    python = ensure_py7zr(root)
    extraction_root.mkdir(parents=True, exist_ok=True)
    print(f"[extract] {archive}")
    subprocess.run(
        [
            str(python), "-c",
            "import py7zr,sys; py7zr.SevenZipFile(sys.argv[1]).extractall(sys.argv[2])",
            str(archive), str(extraction_root),
        ],
        check=True,
    )
    dataset_root = find_dataset_root(extraction_root)
    if dataset_root is None:
        raise RuntimeError("archive extraction completed but the Courtyard scans were not found")
    print(f"[ready] {dataset_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
