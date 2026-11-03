#!/usr/bin/env python3
"""Download the official NVIDIA Isaac ROS nvblox quickstart bag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import urllib.request


VERSIONS_URL = (
    "https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/"
    "isaac_ros_nvblox_assets/versions"
)
FILE_URL = (
    "https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/"
    "isaac_ros_nvblox_assets/versions/{version}/files/quickstart.tar.gz"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path.home() / ".cache" / "horus" / "nvblox_quickstart",
    )
    parser.add_argument(
        "--release",
        default="4.5",
        help="Isaac ROS major.minor line; defaults to the installed HORUS test environment.",
    )
    parser.add_argument(
        "--asset-version",
        default="3.1.0",
        help=(
            "Official NGC asset version. 3.1.0 is the compact 314 MB quickstart bag; "
            "newer 3.2/4.0 archives currently exceed 9 GB."
        ),
    )
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def semver(value: str) -> tuple[int, int, int] | None:
    match = re.fullmatch(r"(\d+)\.(\d+)\.(\d+)", value)
    return tuple(int(part) for part in match.groups()) if match else None


def select_version(release: str, requested_asset: str | None) -> str:
    requested = tuple(int(part) for part in release.split(".", maxsplit=1))
    if len(requested) != 2:
        raise RuntimeError("--release must be major.minor, for example 4.5")
    with urllib.request.urlopen(VERSIONS_URL, timeout=30.0) as response:
        document = json.load(response)
    available = {
        str(item.get("versionId", ""))
        for item in document.get("recipeVersions", [])
    }
    if requested_asset:
        if requested_asset not in available:
            raise RuntimeError(f"NGC has no nvblox asset version {requested_asset}")
        return requested_asset

    candidates = []
    for item in document.get("recipeVersions", []):
        version_id = str(item.get("versionId", ""))
        parsed = semver(version_id)
        if parsed and parsed[0] == requested[0] and parsed[1] <= requested[1]:
            candidates.append((parsed, version_id))
    if not candidates:
        raise RuntimeError(f"NGC has no nvblox quickstart asset for release {release}")
    return max(candidates)[1]


def find_bag(root: Path) -> Path | None:
    for metadata in root.rglob("metadata.yaml"):
        if any(metadata.parent.glob("*.db3")) or any(metadata.parent.glob("*.mcap")):
            return metadata.parent
    return None


def main() -> int:
    args = parse_args()
    output = args.output.expanduser().resolve()
    existing = find_bag(output) if output.exists() else None
    if existing is not None and not args.force:
        print(existing)
        return 0

    output.mkdir(parents=True, exist_ok=True)
    version = select_version(args.release, args.asset_version)
    archive = output / f"quickstart-{version}.tar.gz"
    url = FILE_URL.format(version=version)
    curl = shutil.which("curl")
    if curl is None:
        raise RuntimeError("curl is required to download the resumable 314 MB NGC asset")
    print(f"[download] NVIDIA Isaac ROS nvblox {version}: {url}")
    subprocess.run(
        [curl, "-fL", "--retry", "5", "--retry-all-errors", "-C", "-", "-o", str(archive), url],
        check=True,
    )
    print(f"[extract] {archive}")
    with tarfile.open(archive, "r:gz") as bundle:
        bundle.extractall(output, filter="data")
    bag = find_bag(output)
    if bag is None:
        raise RuntimeError(f"download completed but no ROS 2 bag was found under {output}")
    manifest = {
        "source": "NVIDIA NGC isaac_ros_nvblox_assets",
        "version": version,
        "url": url,
        "bag": str(bag),
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    archive.unlink(missing_ok=True)
    print(bag)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
