"""Native OctoMap coverage regression, without starting any ROS nodes."""

from pathlib import Path
import importlib.util
import os
import shutil
import subprocess

import pytest


def test_unchanged_relay_build_description_does_not_force_rebuild(tmp_path, monkeypatch):
    source = Path(__file__).resolve().parents[1] / "examples/tools/uav_sim_octomap_marker_relay.py"
    spec = importlib.util.spec_from_file_location("octomap_relay_launcher", source)
    relay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(relay)
    monkeypatch.setattr(relay, "SOURCE_DIR", tmp_path)
    cmake = relay._write_cmake()
    os.utime(cmake, ns=(1000000000, 1000000000))
    assert relay._write_cmake().stat().st_mtime_ns == 1000000000
    cmake.write_text("outdated build description", encoding="utf-8")
    relay._write_cmake()
    assert "add_executable(uav_sim_octomap_marker_relay" in cmake.read_text(encoding="utf-8")


def test_adaptive_octomap_preserves_partially_exposed_faces(tmp_path):
    compiler = shutil.which("g++")
    if not compiler or not Path("/usr/include/octomap/OcTree.h").exists():
        pytest.skip("native OctoMap development headers and g++ are required")
    root = Path(__file__).resolve().parents[1]
    binary = tmp_path / "octomap_surface_test"
    subprocess.run([
        compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(root / "examples/tools"),
        str(root / "tests/cpp/octomap_surface_coverage_test.cpp"),
        "-loctomap", "-loctomath", "-o", str(binary),
    ], check=True, capture_output=True, text=True, timeout=60)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=30)
    assert "PASS 192" in result.stdout
