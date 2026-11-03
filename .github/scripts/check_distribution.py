#!/usr/bin/env python3
"""Install each release artifact outside the checkout and exercise its public API."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import venv


def check(artifact: Path, tests: Path, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    for name in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV"):
        env.pop(name, None)
    env["PYTHONNOUSERSITE"] = "1"
    env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    with tempfile.TemporaryDirectory(prefix="horus-distribution-") as directory:
        root = Path(directory)
        venv.EnvBuilder(with_pip=True).create(root / "venv")
        python = root / "venv" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")

        def run(*args: str) -> None:
            subprocess.run([str(python), *args], cwd=root, env=env, check=True)

        run("-m", "pip", "install", str(artifact))
        run("-m", "pip", "check")
        run("-I", "-c", """
import importlib
import importlib.metadata
import importlib.resources
import pathlib
import sys
import horus
assert pathlib.Path(horus.__file__).resolve().is_relative_to(pathlib.Path(sys.prefix).resolve())
for name in ("robot", "dataviz", "mapping", "teaching", "remote_rendering", "bridge.robot_registry"):
    importlib.import_module("horus." + name)
resources = importlib.resources.files("horus")
for resource in ("py.typed", "remote_rendering/cuda_point_renderer.cu", "remote_rendering/cuda_mesh_renderer.cu"):
    assert resources.joinpath(resource).is_file(), resource
entrypoints = importlib.metadata.distribution("horus-sdk").entry_points
assert any(ep.name == "horus-sdk" and callable(ep.load()) for ep in entrypoints)
""")
        copied = root / "python" / "tests"
        copied.mkdir(parents=True)
        fixtures = root / "contracts" / "fixtures"
        fixtures.mkdir(parents=True)
        shutil.copy2(tests.parents[1] / "contracts" / "fixtures" / "teaching.json", fixtures / "teaching.json")
        for name in (
            "test_public_api_compatibility.py",
            "test_costmap_visualizations_payload.py",
            "test_costmap_reconstruction.py",
            "test_teaching.py",
            "test_teach_input_v2.py",
        ):
            shutil.copy2(tests / name, copied / name)
        run("-m", "pip", "install", str(artifact) + "[test,teaching-motion]")
        run("-m", "pip", "check")
        run(
            "-m", "pytest", str(copied), "-q", "-o", "addopts=",
            "--junitxml=" + str(output / (artifact.name + ".xml")),
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", nargs="+", type=Path)
    parser.add_argument("--tests", type=Path, default=Path(__file__).resolve().parents[2] / "python/tests")
    parser.add_argument("--output", type=Path, default=Path("test-results/distributions"))
    args = parser.parse_args()
    for artifact in args.artifacts:
        if not artifact.is_file() or not (artifact.name.endswith(".whl") or artifact.name.endswith(".tar.gz")):
            parser.error(f"Not a wheel or source distribution: {artifact}")
        check(artifact.resolve(), args.tests.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
