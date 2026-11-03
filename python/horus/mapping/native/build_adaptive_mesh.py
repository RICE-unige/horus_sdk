"""Offline build of packaged meshoptimizer sources into the user's cache."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import platform
import shlex
import subprocess
import tempfile


def cache_directory() -> Path:
    return Path(os.environ.get("XDG_CACHE_HOME", "~/.cache")).expanduser() / "horus/adaptive_mesh"


def library_path() -> Path:
    directory = Path(__file__).resolve().parent
    vendor = directory / "vendor"
    sources = [directory / "adaptive_mesh.cpp"] + [
        vendor / name for name in
        ("meshoptimizer.h", "simplifier.cpp", "allocator.cpp", "indexgenerator.cpp")
    ]
    digest = hashlib.sha256()
    digest.update((platform.system() + platform.machine() + platform.libc_ver()[0] +
                   platform.libc_ver()[1] + "-O2-c++17-v1").encode())
    for source in sources:
        digest.update(source.read_bytes())
    return cache_directory() / ("_adaptive_mesh_" + digest.hexdigest()[:20] + ".so")


def build_library() -> Path:
    destination = library_path()
    if destination.is_file():
        destination.chmod(0o755)
        return destination
    directory = Path(__file__).resolve().parent
    vendor = directory / "vendor"
    compiler = shlex.split(os.environ.get("CXX", "c++"))
    if not compiler:
        raise RuntimeError("CXX does not name a C++ compiler")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=destination.parent, suffix=".so", delete=False) as output:
        temporary = Path(output.name)
    try:
        command = compiler + [
            "-std=c++17", "-O2", "-fPIC", "-shared", "-I", str(vendor),
            str(directory / "adaptive_mesh.cpp"),
            *(str(vendor / name) for name in
              ("simplifier.cpp", "allocator.cpp", "indexgenerator.cpp")),
            "-o", str(temporary),
        ]
        result = subprocess.run(command, capture_output=True, text=True, timeout=180)
        if result.returncode:
            raise RuntimeError("Adaptive mesh native build failed:\n" + result.stderr[-8000:])
        temporary.chmod(0o755)
        os.replace(temporary, destination)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RuntimeError(f"Adaptive mesh native preflight failed: {error}") from error
    finally:
        temporary.unlink(missing_ok=True)
    return destination


if __name__ == "__main__":
    print(build_library())
