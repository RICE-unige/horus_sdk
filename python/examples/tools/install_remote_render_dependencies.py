#!/usr/bin/env python3
"""Describe optional GPU dependencies; private-venv installation is opt-in."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path
from collections.abc import Sequence


TORCH_VERSION = "2.11.0"
TORCH_INDEX = "https://download.pytorch.org/whl/cu130"
NVDIFFRAST_URL = (
    "git+https://github.com/NVlabs/nvdiffrast.git@"
    "253ac4fcea7de5f396371124af597e6cc957bfae"
)
PACKAGES = (
    "gsplat==1.5.3",
    "plyfile==1.1.3",
    "ninja==1.13.0",
    "trimesh==4.9.0",
)
PYNVVIDEO_CODEC_VERSION = "2.2.3"


def dependency_plan() -> dict:
    root = Path(os.environ.get("HORUS_REMOTE_CACHE_ROOT", str(
        Path.home() / ".cache/horus/remote_render/isaacsim-6.2-py312-torch2.11-cu130"))).expanduser()
    nvenc = Path(os.environ.get("HORUS_NVENC_ROOT", str(root / "nvenc-color-vui-v1"))).expanduser()
    return {
        "mode": "plan_only",
        "python": str(Path.home() / "isaac/env_isaacsim/bin/python"),
        "shared_stack": {"python": "3.12", "torch": TORCH_VERSION + "+cu130",
                         "torch_index": TORCH_INDEX, "isaacsim": "6.2.0.0"},
        "overlay": str(root / "site-packages"),
        "packages": list(PACKAGES),
        "non_torch_runtime_dependencies": ["numpy>=1.21.0", "rich>=13.0.0",
                                           "opencv-python-headless>=4.8,<5", "pillow>=9", "jaxtyping"],
        "ros_imports": ["rclpy", "sensor_msgs", "std_msgs", "visualization_msgs"],
        "overlay_install_policy": "Use --no-deps for overlay wheels/builds; the coordinator supplies compatible dependencies without replacing shared Torch.",
        "nvdiffrast_source": NVDIFFRAST_URL,
        "nvdiffrast_build": "Build from source against the canonical Torch/CUDA; do not copy an old extension.",
        "torch_extensions": str(root / "torch-extensions"),
        "native_renderer_cache": str(root / "native"),
        "nvenc": {"source_version": "2.2.2", "root": str(nvenc),
                  "source_url": "https://api.ngc.nvidia.com/v2/resources/nvidia/pynvvideocodec/versions/2.2.2/files/PyNvVideoCodec_2.2.2.zip",
                  "source_sha256": "0ecb47da0bd40dc70e1e684fa95073fd6f3debc607fd4a5056b99b42af5cbeb1",
                  "build_dependencies": ["setuptools>=42", "wheel", "scikit-build", "cmake", "ninja", "numpy"],
                  "build_script": str(Path(__file__).with_name("install_remote_nvenc.py")),
                  "requires": ["extra_output_delay", "horus_h264_profile", "horus_h264_color_range"],
                  "stock_package_is_not_a_substitute": True},
        "validation": "GPU/backend acceptance pending; this command imports no GPU runtime and installs nothing.",
    }


def require_private_environment() -> None:
    prefix = Path(sys.prefix).resolve()
    isaac_root = (Path.home() / "isaac").resolve()
    shared_prefixes = {(isaac_root / name).resolve()
                       for name in ("env_isaacsim", "env_isaacsim_61")}
    if prefix in shared_prefixes or prefix.is_relative_to(isaac_root) or os.environ.get("ISAACLAB_PATH"):
        raise RuntimeError("Shared Isaac/Lab dependencies are coordinator-managed; use --plan, not --install.")
    if sys.prefix == sys.base_prefix:
        raise RuntimeError("Installation requires a dedicated private HORUS virtual environment; use --plan for the shared runtime.")


def _pip_install(arguments: Sequence[str]) -> None:
    command = [
        sys.executable,
        "-m",
        "pip",
        "install",
        *arguments,
    ]
    print("[remote-render-deps]", " ".join(command), flush=True)
    subprocess.run(command, check=True)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--plan", action="store_true", help="Print the central dependency/rebuild plan without installing (default).")
    mode.add_argument("--install", action="store_true", help="Install only into an explicitly selected private HORUS venv, never Isaac/Lab.")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)
    if args.force and not args.install:
        parser.error("--force requires --install and a private HORUS virtual environment")
    if not args.install:
        print(json.dumps(dependency_plan(), indent=2))
        return 0
    try:
        require_private_environment()
    except RuntimeError as exc:
        parser.error(str(exc))

    if args.force or importlib.util.find_spec("torch") is None:
        _pip_install(
            (
                "--index-url",
                TORCH_INDEX,
                f"torch=={TORCH_VERSION}",
            )
        )
    if args.force or any(
        importlib.util.find_spec(name) is None
        for name in ("gsplat", "plyfile", "trimesh")
    ):
        _pip_install(list(PACKAGES))
    if args.force or importlib.util.find_spec("nvdiffrast") is None:
        _pip_install(("--no-build-isolation", NVDIFFRAST_URL))
    if args.force or importlib.util.find_spec("PyNvVideoCodec") is None:
        _pip_install((f"PyNvVideoCodec=={PYNVVIDEO_CODEC_VERSION}",))

    nvenc_target = Path(os.environ.get("HORUS_NVENC_ROOT", str(
        Path.home() / ".cache/horus/remote_render/nvenc-low-latency-v2"))).expanduser() / "site-packages/PyNvVideoCodec"
    if args.force or not nvenc_target.is_dir():
        subprocess.run([sys.executable, str(Path(__file__).with_name("install_remote_nvenc.py"))], check=True)

    import torch

    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch installed, but no CUDA-capable GPU is available")
    print(
        "[remote-render-deps] ready "
        f"torch={torch.__version__} cuda={torch.version.cuda} "
        f"gpu={torch.cuda.get_device_name(0)}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
