#!/usr/bin/env python3
"""Build and run the UAV sim Octomap-to-Marker relay.

HORUS MR currently renders octomap registrations through MeshMarkerVisualizer,
which subscribes to visualization_msgs/Marker TRIANGLE_LIST messages. This tool
keeps the UAV demo compatible by converting the sim's octomap_msgs/Octomap topic
to that marker format at runtime. The default style emits the full exposed
surface shell so HORUS receives the whole map with fewer triangles than full
occupied voxel cubes.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


SOURCE = Path(__file__).with_suffix(".cpp")
CACHE_ROOT = Path.home() / ".cache" / "horus_sdk" / "uav_sim_octomap_marker_relay"
BUILD_DIR = CACHE_ROOT / "build"
SOURCE_DIR = CACHE_ROOT / "src"
BINARY = BUILD_DIR / "uav_sim_octomap_marker_relay"


def _write_cmake() -> Path:
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    cmake_path = SOURCE_DIR / "CMakeLists.txt"
    cmake_path.write_text(
        f"""
cmake_minimum_required(VERSION 3.16)
project(horus_uav_octomap_marker_relay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(uav_sim_octomap_marker_relay "{SOURCE.as_posix()}")
target_compile_features(uav_sim_octomap_marker_relay PRIVATE cxx_std_17)
ament_target_dependencies(
  uav_sim_octomap_marker_relay
  rclcpp
  octomap_msgs
  visualization_msgs
  geometry_msgs
)
target_include_directories(uav_sim_octomap_marker_relay PRIVATE ${{OCTOMAP_INCLUDE_DIRS}})
target_link_libraries(uav_sim_octomap_marker_relay ${{OCTOMAP_LIBRARIES}})
""".strip()
        + "\n",
        encoding="utf-8",
    )
    return cmake_path


def _needs_rebuild(cmake_path: Path) -> bool:
    if not BINARY.exists():
        return True
    binary_mtime = BINARY.stat().st_mtime
    return SOURCE.stat().st_mtime > binary_mtime or cmake_path.stat().st_mtime > binary_mtime


def ensure_built() -> None:
    if not SOURCE.exists():
        raise FileNotFoundError(f"missing C++ source: {SOURCE}")
    if shutil.which("cmake") is None:
        raise RuntimeError("cmake is required to build the octomap marker relay")
    if shutil.which("g++") is None:
        raise RuntimeError("g++ is required to build the octomap marker relay")

    cmake_path = _write_cmake()
    if not _needs_rebuild(cmake_path):
        return

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["cmake", "-S", str(SOURCE_DIR), "-B", str(BUILD_DIR), "-DCMAKE_BUILD_TYPE=Release"],
        check=True,
    )
    subprocess.run(
        ["cmake", "--build", str(BUILD_DIR), "--target", "uav_sim_octomap_marker_relay", "-j2"],
        check=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--build-only", action="store_true")
    known, remaining = parser.parse_known_args()

    ensure_built()
    if known.build_only:
        print(BINARY)
        return

    os.execv(str(BINARY), [str(BINARY), *remaining])


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        raise SystemExit(exc.returncode) from exc
    except Exception as exc:
        print(f"uav_sim_octomap_marker_relay: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
