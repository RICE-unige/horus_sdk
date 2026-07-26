#!/usr/bin/env python3
"""Install the optional PC GPU dependencies used by advanced remote maps."""

from __future__ import annotations

import argparse
import importlib.util
import subprocess
import sys
from collections.abc import Sequence


TORCH_VERSION = "2.12.1"
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


def _pip_install(arguments: Sequence[str]) -> None:
    command = [
        sys.executable,
        "-m",
        "pip",
        "install",
        "--user",
        "--break-system-packages",
        *arguments,
    ]
    print("[remote-render-deps]", " ".join(command), flush=True)
    subprocess.run(command, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

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
