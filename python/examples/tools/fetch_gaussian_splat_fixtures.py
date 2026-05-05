#!/usr/bin/env python3
"""Download Gaussian Splatting fixture datasets for HORUS DataViz work.

The files are stored outside the repository by default:
    ~/.cache/horus/gaussian_splatting
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path
from typing import Dict, List, Set


DEFAULT_ROOT = Path("~/.cache/horus/gaussian_splatting").expanduser()


VOXEL51_BASE = "https://huggingface.co/datasets/Voxel51/gaussian_splatting/resolve/main"
NERFSTUDIO_BASE = "https://huggingface.co/datasets/nerfstudioteam/datasets/resolve/main"


VOXEL51_SCENES: Dict[str, Dict[str, str]] = {
    "drjohnson": {
        "reference_image": f"{VOXEL51_BASE}/FO_dataset/drjohnson/IMG_6292.jpg?download=true",
        "splat_7000": f"{VOXEL51_BASE}/FO_dataset/drjohnson/point_cloud/iteration_7000/point_cloud.ply?download=true",
        "splat_30000": f"{VOXEL51_BASE}/FO_dataset/drjohnson/point_cloud/iteration_30000/point_cloud.ply?download=true",
    },
    "playroom": {
        "reference_image": f"{VOXEL51_BASE}/FO_dataset/playroom/DSC05572.jpg?download=true",
        "splat_7000": f"{VOXEL51_BASE}/FO_dataset/playroom/point_cloud/iteration_7000/point_cloud.ply?download=true",
        "splat_30000": f"{VOXEL51_BASE}/FO_dataset/playroom/point_cloud/iteration_30000/point_cloud.ply?download=true",
    },
    "train": {
        "reference_image": f"{VOXEL51_BASE}/FO_dataset/train/00001.jpg?download=true",
        "splat_7000": f"{VOXEL51_BASE}/FO_dataset/train/point_cloud/iteration_7000/point_cloud.ply?download=true",
        "splat_30000": f"{VOXEL51_BASE}/FO_dataset/train/point_cloud/iteration_30000/point_cloud.ply?download=true",
    },
    "truck": {
        "reference_image": f"{VOXEL51_BASE}/FO_dataset/truck/000001.jpg?download=true",
        "splat_7000": f"{VOXEL51_BASE}/FO_dataset/truck/point_cloud/iteration_7000/point_cloud.ply?download=true",
        "splat_30000": f"{VOXEL51_BASE}/FO_dataset/truck/point_cloud/iteration_30000/point_cloud.ply?download=true",
    },
}


POSTER_REQUIRED_FILES = [
    "poster/sparse_pc.ply",
    "poster/base_cam.json",
    "poster/colmap/database.db",
    "poster/colmap/sparse/0/cameras.bin",
    "poster/colmap/sparse/0/images.bin",
    "poster/colmap/sparse/0/points3D.bin",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=DEFAULT_ROOT,
        help=f"Fixture cache root. Default: {DEFAULT_ROOT}",
    )
    parser.add_argument(
        "--bundle",
        choices=("prebuilt", "poster-lite", "all"),
        default="prebuilt",
        help=(
            "prebuilt downloads one official-format Gaussian Splat PLY; "
            "poster-lite downloads the Nerfstudio poster image/COLMAP dataset; "
            "all downloads both."
        ),
    )
    parser.add_argument(
        "--scene",
        choices=sorted(VOXEL51_SCENES),
        default="playroom",
        help="Voxel51 prebuilt scene to download.",
    )
    parser.add_argument(
        "--iteration",
        choices=("7000", "30000"),
        default="7000",
        help="Prebuilt Gaussian Splat iteration. 7000 is smaller/faster for first tests.",
    )
    parser.add_argument(
        "--poster-resolution",
        choices=("images_8", "images_4", "images_2", "images"),
        default="images_8",
        help="Nerfstudio poster image folder to download. images_8 is tiny and enough for pipeline tests.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Re-download existing files.",
    )
    return parser.parse_args()


def download_file(url: str, dest: Path, force: bool = False) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and dest.stat().st_size > 0 and not force:
        print(f"[skip] {dest}")
        return

    tmp = dest.with_suffix(dest.suffix + ".part")
    if tmp.exists():
        tmp.unlink()

    print(f"[get] {url}")
    with urllib.request.urlopen(url) as response, tmp.open("wb") as out:
        length_header = response.headers.get("Content-Length")
        total = int(length_header) if length_header and length_header.isdigit() else 0
        read = 0
        last_report = time.time()
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            out.write(chunk)
            read += len(chunk)
            now = time.time()
            if total > 0 and now - last_report >= 2.0:
                print(f"      {read / total:5.1%} ({read / (1024 * 1024):.1f} MiB)")
                last_report = now

    tmp.replace(dest)
    print(f"[ok]  {dest} ({dest.stat().st_size / (1024 * 1024):.1f} MiB)")


def list_huggingface_folder_files(rel_folder: str) -> Set[str]:
    api_url = f"https://huggingface.co/api/datasets/nerfstudioteam/datasets/tree/main/{rel_folder}?recursive=1&expand=1"
    with urllib.request.urlopen(api_url) as response:
        entries = json.load(response)

    names: Set[str] = set()
    for entry in entries:
        if entry.get("type") != "file":
            continue
        path_value = str(entry.get("path", ""))
        if path_value:
            names.add(Path(path_value).name)
    return names


def download_prebuilt(args: argparse.Namespace, manifest: Dict) -> None:
    scene = args.scene
    iteration = args.iteration
    scene_dir = args.root.expanduser() / "prebuilt" / scene
    urls = VOXEL51_SCENES[scene]

    splat_key = f"splat_{iteration}"
    splat_path = scene_dir / f"{scene}_{iteration}.ply"
    ref_path = scene_dir / "reference_image.png"

    download_file(urls[splat_key], splat_path, force=args.force)
    try:
        download_file(urls["reference_image"], ref_path, force=args.force)
    except Exception as exc:
        print(f"[warn] reference image download failed: {exc}")

    manifest["prebuilt"] = {
        "dataset": "Voxel51/gaussian_splatting",
        "license": "apache-2.0",
        "scene": scene,
        "iteration": int(iteration),
        "splat_ply": str(splat_path),
        "reference_image": str(ref_path) if ref_path.exists() else "",
        "source_url": "https://huggingface.co/datasets/Voxel51/gaussian_splatting",
        "recommended_preview_topic": "/map_gaussian_splat_preview",
        "recommended_manifest_topic": "/horus/gaussian_splat/manifest",
    }


def download_poster_lite(args: argparse.Namespace, manifest: Dict) -> None:
    root = args.root.expanduser() / "nerfstudio" / "poster"
    source_transforms_path = root / "transforms_source.json"
    download_file(
        f"{NERFSTUDIO_BASE}/poster/transforms.json?download=true",
        source_transforms_path,
        force=args.force,
    )

    for rel in POSTER_REQUIRED_FILES:
        download_file(f"{NERFSTUDIO_BASE}/{rel}?download=true", root / Path(rel).relative_to("poster"), force=args.force)

    image_folder = args.poster_resolution
    available_images = list_huggingface_folder_files(f"poster/{image_folder}")
    if not available_images:
        raise RuntimeError(f"No images found in Hugging Face folder poster/{image_folder}")

    try:
        with source_transforms_path.open("r", encoding="utf-8") as handle:
            transforms = json.load(handle)
        source_frame_names: List[str] = []
        filtered_frames: List[Dict] = []
        for frame in transforms.get("frames", []):
            file_path = str(frame.get("file_path", "")).strip()
            if not file_path:
                continue
            filename = Path(file_path).name
            if filename:
                source_frame_names.append(filename)
                if filename in available_images:
                    filtered_frame = dict(frame)
                    filtered_frame["file_path"] = f"./{image_folder}/{filename}"
                    filtered_frames.append(filtered_frame)
        frame_paths = sorted({Path(str(frame.get("file_path", ""))).name for frame in filtered_frames})
    except Exception as exc:
        raise RuntimeError(f"Failed to parse poster transforms at {source_transforms_path}: {exc}") from exc

    if not frame_paths:
        raise RuntimeError(f"No poster frame paths found in {source_transforms_path}")

    missing_count = len(set(source_frame_names) - set(frame_paths))
    if missing_count:
        print(
            f"[info] poster/{image_folder} contains {len(frame_paths)} of "
            f"{len(set(source_frame_names))} transform frames; writing a filtered transforms.json"
        )

    for filename in frame_paths:
        rel = f"poster/{image_folder}/{filename}"
        download_file(f"{NERFSTUDIO_BASE}/{rel}?download=true", root / image_folder / filename, force=args.force)

    filtered_transforms = dict(transforms)
    filtered_transforms["frames"] = filtered_frames
    transforms_path = root / "transforms.json"
    with transforms_path.open("w", encoding="utf-8") as handle:
        json.dump(filtered_transforms, handle, indent=2)
        handle.write("\n")

    manifest["poster_lite"] = {
        "dataset": "nerfstudioteam/datasets/poster",
        "license": "see Hugging Face dataset card",
        "path": str(root),
        "image_folder": image_folder,
        "image_count": len(frame_paths),
        "transforms_json": str(transforms_path),
        "source_transforms_json": str(source_transforms_path),
        "colmap_dir": str(root / "colmap"),
        "sparse_point_cloud": str(root / "sparse_pc.ply"),
        "source_url": "https://huggingface.co/datasets/nerfstudioteam/datasets/tree/main/poster",
        "nerfstudio_train_command": (
            f"ns-train splatfacto --data {root} --output-dir "
            f"{args.root.expanduser() / 'outputs'}"
        ),
    }


def write_manifest(root: Path, manifest: Dict) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    manifest_path = root / "manifest.json"
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")
    return manifest_path


def load_existing_manifest(root: Path) -> Dict:
    manifest_path = root / "manifest.json"
    if not manifest_path.exists():
        return {}
    try:
        with manifest_path.open("r", encoding="utf-8") as handle:
            loaded = json.load(handle)
        return loaded if isinstance(loaded, dict) else {}
    except Exception as exc:
        print(f"[warn] ignoring unreadable existing manifest {manifest_path}: {exc}")
        return {}


def main() -> int:
    args = parse_args()
    args.root = args.root.expanduser()
    manifest: Dict = load_existing_manifest(args.root)
    manifest["generated_by"] = Path(__file__).name
    manifest["cache_root"] = str(args.root)

    if args.bundle in ("prebuilt", "all"):
        download_prebuilt(args, manifest)
    if args.bundle in ("poster-lite", "all"):
        download_poster_lite(args, manifest)

    manifest_path = write_manifest(args.root, manifest)
    print(f"\nManifest written: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
