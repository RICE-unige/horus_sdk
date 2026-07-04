#!/usr/bin/env python3
"""Download the ETHZ ASL Voxblox Cow & Lady dataset for HORUS mesh-map demos.

The ROS bag is large, so this tool stores it outside the repository by default:

    ~/.cache/horus/voxblox_cow_lady
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path


DEFAULT_ROOT = Path("~/.cache/horus/voxblox_cow_lady").expanduser()
DATA_BAG_URLS = [
    "https://www.research-collection.ethz.ch/server/api/core/bitstreams/bfb68f88-fcb2-4e09-aa53-434d9162cef5/content",
    "http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/data.bag",
]
EXTRAS_URLS = [
    "https://www.research-collection.ethz.ch/server/api/core/bitstreams/0eb17372-0367-4527-9c59-8ec0a1ea9905/content",
    "http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/voxblox_cow_extras.zip",
]
DATASET_PAGE = "https://projects.asl.ethz.ch/datasets/voxblox/"
RESEARCH_COLLECTION_PAGE = (
    "https://www.research-collection.ethz.ch/entities/researchdata/"
    "ded5ea04-6ec7-42e9-a5ca-6062cf83507c"
)
DATASET_DOI = "10.3929/ethz-b-000721636"
DEFAULT_RETRIES = 8
CHUNK_BYTES = 1024 * 1024


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=DEFAULT_ROOT,
        help=f"Download/cache directory. Default: {DEFAULT_ROOT}",
    )
    parser.add_argument("--bag-only", action="store_true", help="Download only data.bag.")
    parser.add_argument("--extras-only", action="store_true", help="Download only voxblox_cow_extras.zip.")
    parser.add_argument("--force", action="store_true", help="Re-download existing files.")
    parser.add_argument(
        "--download-tool",
        choices=("auto", "curl", "wget", "python"),
        default="auto",
        help="Downloader to use. Default: auto.",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=DEFAULT_RETRIES,
        help=f"Retry attempts per URL for the Python downloader. Default: {DEFAULT_RETRIES}",
    )
    return parser.parse_args()


def choose_download_tool(preferred: str) -> str:
    if preferred != "auto":
        return preferred
    for candidate in ("curl", "wget"):
        if shutil.which(candidate):
            return candidate
    return "python"


def run_external_downloader(tool: str, url: str, tmp: Path) -> None:
    if tool == "curl":
        command = [
            "curl",
            "-L",
            "--fail",
            "--retry",
            "12",
            "--retry-delay",
            "5",
            "--connect-timeout",
            "30",
            "--speed-time",
            "60",
            "--speed-limit",
            "1024",
            "-C",
            "-",
            "-o",
            str(tmp),
            url,
        ]
    elif tool == "wget":
        command = [
            "wget",
            "-c",
            "--tries=12",
            "--timeout=60",
            "--read-timeout=60",
            "-O",
            str(tmp),
            url,
        ]
    else:
        raise ValueError(f"Unsupported external downloader: {tool}")

    subprocess.run(command, check=True)


def download_with_python(url: str, tmp: Path, retries: int) -> None:
    for attempt in range(1, retries + 1):
        resume_from = tmp.stat().st_size if tmp.exists() else 0
        request = urllib.request.Request(url)
        if resume_from > 0:
            request.add_header("Range", f"bytes={resume_from}-")

        try:
            print(f"[get] {url} attempt={attempt}/{retries} resume={resume_from / (1024 * 1024):.1f} MiB")
            with urllib.request.urlopen(request, timeout=120.0) as response, tmp.open("ab") as out:
                if resume_from > 0 and response.status == 200:
                    print("[warn] server ignored resume request; restarting partial download")
                    tmp.unlink(missing_ok=True)
                    return download_with_python(url, tmp, retries - attempt + 1)

                length_header = response.headers.get("Content-Length")
                remaining = int(length_header) if length_header and length_header.isdigit() else 0
                total = resume_from + remaining if remaining > 0 else 0
                read = resume_from
                last_report = time.monotonic()
                while True:
                    chunk = response.read(CHUNK_BYTES)
                    if not chunk:
                        break
                    out.write(chunk)
                    read += len(chunk)
                    now = time.monotonic()
                    if now - last_report >= 2.0:
                        if total > 0:
                            print(f"      {read / total:5.1%} ({read / (1024 * 1024):.1f} MiB)")
                        else:
                            print(f"      {read / (1024 * 1024):.1f} MiB")
                        last_report = now
            return
        except (TimeoutError, urllib.error.URLError, ConnectionError) as exc:
            if attempt >= retries:
                raise
            wait = min(5 * attempt, 30)
            print(f"[retry] {exc}; waiting {wait}s")
            time.sleep(wait)


def download_file(
    urls: list[str],
    dest: Path,
    force: bool = False,
    tool: str = "auto",
    retries: int = DEFAULT_RETRIES,
) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and dest.stat().st_size > 0 and not force:
        print(f"[skip] {dest} ({dest.stat().st_size / (1024 * 1024):.1f} MiB)")
        return

    tmp = dest.with_suffix(dest.suffix + ".part")
    if tmp.exists() and force:
        tmp.unlink()

    selected_tool = choose_download_tool(tool)
    failures: list[str] = []
    for url in urls:
        try:
            print(f"[get] {url} via {selected_tool}")
            if selected_tool in ("curl", "wget"):
                run_external_downloader(selected_tool, url, tmp)
            else:
                download_with_python(url, tmp, retries)
            break
        except (subprocess.CalledProcessError, OSError, urllib.error.URLError, TimeoutError) as exc:
            failures.append(f"{url}: {exc}")
            print(f"[fail] {url}: {exc}")
    else:
        message = "\n  - ".join(failures)
        raise RuntimeError(
            f"Failed to download {dest.name}. Partial data is preserved at {tmp}.\n"
            f"Retry with: python3 python/examples/tools/fetch_voxblox_cow_lady.py\n"
            f"Failures:\n  - {message}"
        )

    tmp.replace(dest)
    print(f"[ok]  {dest} ({dest.stat().st_size / (1024 * 1024):.1f} MiB)")


def write_manifest(root: Path) -> None:
    manifest = {
        "dataset": "Voxblox Cow & Lady",
        "source": DATASET_PAGE,
        "research_collection": RESEARCH_COLLECTION_PAGE,
        "doi": DATASET_DOI,
        "bag": str(root / "data.bag"),
        "extras": str(root / "voxblox_cow_extras.zip"),
        "notes": (
            "For ROS 2-native HORUS demos, unzip extras and publish cow_and_lady_gt.ply "
            "with python/examples/tools/ply_to_horus_mesh_marker.py. For upstream Voxblox, "
            "run Voxblox on data.bag, relay /voxblox_node/mesh with "
            "python/examples/tools/voxblox_mesh_to_horus_marker.py, and register "
            "python/examples/mesh_map_registration.py."
        ),
    }
    path = root / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"[manifest] {path}")


def main() -> None:
    args = parse_args()
    root = args.root.expanduser()
    if args.bag_only and args.extras_only:
        raise SystemExit("Choose at most one of --bag-only or --extras-only.")

    if not args.extras_only:
        download_file(
            DATA_BAG_URLS,
            root / "data.bag",
            force=args.force,
            tool=args.download_tool,
            retries=args.retries,
        )
    if not args.bag_only:
        download_file(
            EXTRAS_URLS,
            root / "voxblox_cow_extras.zip",
            force=args.force,
            tool=args.download_tool,
            retries=args.retries,
        )
    write_manifest(root)


if __name__ == "__main__":
    main()
