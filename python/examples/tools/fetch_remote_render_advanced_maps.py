#!/usr/bin/env python3
"""Download licensed mesh and Gaussian Splat remote-render showcases."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import urllib.request
import zipfile


SPONZA_API = (
    "https://api.github.com/repos/KhronosGroup/glTF-Sample-Assets/"
    "contents/Models/Sponza/glTF?ref=main"
)
SPONZA_METADATA_BASE = (
    "https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/"
    "main/Models/Sponza"
)
SAN_MIGUEL_URL = (
    "https://casual-effects.com/g3d/data10/research/model/"
    "San_Miguel/San_Miguel.zip"
)
GAUSSIAN_BASE = (
    "https://huggingface.co/datasets/Voxel51/gaussian_splatting/resolve/main/"
    "FO_dataset"
)
GAUSSIAN_ASSETS = {
    "gaussian_drjohnson": {
        "folder": "drjohnson",
        "reference": "IMG_6292.jpg",
        "ply_size": 788_034_924,
        "reference_size": 603_709,
    },
    "gaussian_playroom": {
        "folder": "playroom",
        "reference": "DSC05572.jpg",
        "ply_size": 475_263_524,
        "reference_size": 563_323,
    },
    "gaussian_train": {
        "folder": "train",
        "reference": "00001.jpg",
        "ply_size": 266_542_260,
        "reference_size": 372_211,
    },
    "gaussian_truck": {
        "folder": "truck",
        "reference": "000001.jpg",
        "ply_size": 510_049_492,
        "reference_size": 465_332,
    },
}
MESH_SCENE_IDS = ("sponza_mesh", "san_miguel_mesh")
GAUSSIAN_SCENE_IDS = tuple(GAUSSIAN_ASSETS)
ALL_SCENE_IDS = (*MESH_SCENE_IDS, *GAUSSIAN_SCENE_IDS)
DEFAULT_ROOT = Path.home() / ".cache" / "horus"


def _download(url: str, destination: Path, *, force: bool, size: int = 0) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if (
        destination.is_file()
        and destination.stat().st_size > 0
        and not force
        and (size <= 0 or destination.stat().st_size == size)
    ):
        print(f"[skip] {destination}")
        return
    curl = shutil.which("curl")
    if not curl:
        raise RuntimeError("curl is required to download the remote-render assets")
    temporary = destination.with_suffix(destination.suffix + ".part")
    command = [
        curl,
        "--fail",
        "--location",
        "--retry",
        "5",
        "--retry-all-errors",
        "--connect-timeout",
        "30",
        "--continue-at",
        "-",
        "--user-agent",
        "HORUS remote-render asset fetcher",
        "--output",
        str(temporary),
        url,
    ]
    print(f"[get]  {url}", flush=True)
    subprocess.run(command, check=True)
    if size > 0 and temporary.stat().st_size != size:
        raise RuntimeError(
            f"download size mismatch for {destination.name}: "
            f"{temporary.stat().st_size} != {size}"
        )
    temporary.replace(destination)
    print(f"[ok]   {destination} ({destination.stat().st_size / 1048576:.1f} MiB)")


def _safe_extract(archive: Path, destination: Path, *, force: bool) -> None:
    marker = destination / ".extracted"
    if marker.is_file() and not force:
        return
    destination.mkdir(parents=True, exist_ok=True)
    root = destination.resolve()
    with zipfile.ZipFile(archive) as source:
        for entry in source.infolist():
            target = (destination / entry.filename).resolve()
            if root not in target.parents and target != root:
                raise RuntimeError(f"unsafe archive member: {entry.filename}")
        source.extractall(destination)
    marker.write_text(archive.name + "\n", encoding="utf-8")


def _fetch_sponza(root: Path, force: bool) -> dict:
    destination = root / "remote_render_assets" / "mesh" / "sponza"
    request = urllib.request.Request(
        SPONZA_API,
        headers={"Accept": "application/vnd.github+json"},
    )
    with urllib.request.urlopen(request, timeout=60.0) as response:
        entries = json.load(response)
    for entry in entries:
        if entry.get("type") != "file":
            continue
        _download(
            str(entry["download_url"]),
            destination / str(entry["name"]),
            force=force,
            size=int(entry.get("size") or 0),
        )
    for name in ("LICENSE.md", "README.md"):
        _download(
            f"{SPONZA_METADATA_BASE}/{name}",
            destination / name,
            force=force,
        )
    return {
        "name": "Sponza",
        "scene_id": "sponza_mesh",
        "kind": "textured_triangle_mesh",
        "path": str(destination / "Sponza.gltf"),
        "source": (
            "https://github.com/KhronosGroup/glTF-Sample-Assets/"
            "tree/main/Models/Sponza"
        ),
        "license_file": str(destination / "LICENSE.md"),
    }


def _fetch_san_miguel(root: Path, force: bool) -> dict:
    destination = root / "remote_render_assets" / "mesh" / "san_miguel"
    archive = destination.parent / "San_Miguel.zip"
    _download(SAN_MIGUEL_URL, archive, force=force)
    _safe_extract(archive, destination, force=force)
    candidates = sorted(
        (
            path
            for path in destination.rglob("*.obj")
            if "low" not in path.name.lower()
        ),
        key=lambda path: path.stat().st_size,
        reverse=True,
    )
    if not candidates:
        raise RuntimeError(f"San Miguel archive contains no full-resolution OBJ: {archive}")
    return {
        "name": "San Miguel 2.0",
        "scene_id": "san_miguel_mesh",
        "kind": "textured_triangle_mesh",
        "path": str(candidates[0]),
        "source": "https://casual-effects.com/data",
        "license": "See the license bundled with the downloaded asset",
    }


def _fetch_mesh(scene_id: str, root: Path, force: bool) -> dict:
    if scene_id == "sponza_mesh":
        return _fetch_sponza(root, force)
    if scene_id == "san_miguel_mesh":
        return _fetch_san_miguel(root, force)
    raise ValueError(f"not a mesh scene: {scene_id}")


def _fetch_gaussian(scene_id: str, root: Path, force: bool) -> dict:
    asset = GAUSSIAN_ASSETS[scene_id]
    folder = str(asset["folder"])
    destination = root / "gaussian_splatting" / "prebuilt" / folder
    ply_path = destination / f"{folder}_30000.ply"
    reference_path = destination / "reference_image.jpg"
    _download(
        f"{GAUSSIAN_BASE}/{folder}/point_cloud/iteration_30000/"
        "point_cloud.ply?download=true",
        ply_path,
        force=force,
        size=int(asset["ply_size"]),
    )
    _download(
        f"{GAUSSIAN_BASE}/{folder}/{asset['reference']}?download=true",
        reference_path,
        force=force,
        size=int(asset["reference_size"]),
    )
    return {
        "name": folder,
        "scene_id": scene_id,
        "kind": "anisotropic_gaussian_splat",
        "iteration": 30000,
        "path": str(ply_path),
        "reference_image": str(reference_path),
        "source": "https://huggingface.co/datasets/Voxel51/gaussian_splatting",
        "license": "Apache-2.0",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--asset",
        choices=("mesh", "gaussian", "all"),
        default="all",
        help="Fetch every scene of one kind. Ignored when --scene is supplied.",
    )
    parser.add_argument("--scene", choices=ALL_SCENE_IDS)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    root = args.root.expanduser()
    if args.scene:
        scene_ids = (args.scene,)
    elif args.asset == "mesh":
        scene_ids = MESH_SCENE_IDS
    elif args.asset == "gaussian":
        scene_ids = GAUSSIAN_SCENE_IDS
    else:
        scene_ids = ALL_SCENE_IDS

    manifest = {"generated_by": Path(__file__).name, "scenes": {}}
    for scene_id in scene_ids:
        if scene_id in MESH_SCENE_IDS:
            record = _fetch_mesh(scene_id, root, args.force)
        else:
            record = _fetch_gaussian(scene_id, root, args.force)
        manifest["scenes"][scene_id] = record

    manifest_path = root / "remote_render_assets" / "advanced_maps.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    if manifest_path.is_file():
        try:
            previous = json.loads(manifest_path.read_text(encoding="utf-8"))
            previous.setdefault("scenes", {}).update(manifest["scenes"])
            manifest = previous
            manifest["generated_by"] = Path(__file__).name
        except (OSError, ValueError, TypeError):
            pass
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"[manifest] {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
