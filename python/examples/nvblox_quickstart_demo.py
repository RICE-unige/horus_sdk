#!/usr/bin/env python3
"""Run NVIDIA's nvblox quickstart bag and display its live mesh in HORUS."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
FETCHER = HERE / "tools" / "fetch_nvblox_quickstart.py"
REGISTRATION = HERE / "nvblox_mesh_map_registration.py"
ADAPTER = HERE / "tools" / "nvblox_mesh_to_horus.py"
DEFAULT_CACHE = Path.home() / ".cache" / "horus" / "nvblox_quickstart"


def build_pythonpath(env: dict[str, str]) -> str:
    paths = [str(HERE.parent)]
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    for prefix in env.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        candidate = Path(prefix) / "lib" / version / "site-packages"
        if candidate.is_dir():
            paths.append(str(candidate))
    paths.extend(path for path in env.get("PYTHONPATH", "").split(os.pathsep) if path)
    return os.pathsep.join(dict.fromkeys(paths))


def parse_float(value: str) -> float:
    return float(str(value).strip().replace(",", "."))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--workspace-scale", type=parse_float, default=0.1)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--blocks-per-axis", type=int, default=4)
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "10")))
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--no-loop", action="store_true")
    return parser.parse_args()


def read_bag(root: Path) -> Path | None:
    manifest = root / "manifest.json"
    if manifest.exists():
        data = json.loads(manifest.read_text())
        bag = Path(data["bag"])
        if (bag / "metadata.yaml").exists():
            return bag
    for metadata in root.rglob("metadata.yaml") if root.exists() else ():
        return metadata.parent
    return None


def signal_process_group(process: subprocess.Popen[str], sig: signal.Signals) -> bool:
    try:
        os.killpg(process.pid, sig)
        return True
    except ProcessLookupError:
        return False


def stop_processes(*candidates: subprocess.Popen[str] | None) -> None:
    processes = [process for process in candidates if process is not None]
    if not processes:
        return

    # Signal all children together. Sequential 10-second waits previously let a
    # force-quit interrupt cleanup before the final adapter had been reached.
    for process in processes:
        signal_process_group(process, signal.SIGINT)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and any(process.poll() is None for process in processes):
        time.sleep(0.05)

    # ros2 launch can exit before all component/container children. Those
    # children remain in its process group, so always finish the whole group.
    for process in processes:
        signal_process_group(process, signal.SIGTERM)
    time.sleep(0.25)
    for process in processes:
        signal_process_group(process, signal.SIGKILL)
        try:
            process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            pass


def wait_for_mesh_pages(
    process: subprocess.Popen[str],
    adapter: subprocess.Popen[str],
    adapter_log: Path,
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"NVIDIA nvblox launch exited with code {process.returncode}")
        if adapter.poll() is not None:
            raise RuntimeError(f"HORUS nvblox adapter exited with code {adapter.returncode}")
        text = adapter_log.read_text(errors="replace") if adapter_log.exists() else ""
        if re.search(r'"pages":\s*[1-9]\d*', text):
            return
        time.sleep(0.5)
    raise RuntimeError("timed out waiting for the adapter to retain nvblox mesh pages")


def main() -> int:
    args = parse_args()
    dataset_root = args.dataset_root.expanduser().resolve()
    bag = read_bag(dataset_root)
    if bag is None:
        subprocess.run(
            [sys.executable, str(FETCHER), "--output", str(dataset_root)],
            check=True,
        )
        bag = read_bag(dataset_root)
    if bag is None:
        raise RuntimeError(f"no ROS 2 bag found under {dataset_root}")

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    # Old hard-coded CycloneDDS NIC names are a common source of failed demos after
    # WSL/network restarts. Local nvblox, adapter, SDK, and bridge share this process domain.
    env.pop("CYCLONEDDS_URI", None)
    env["PYTHONPATH"] = build_pythonpath(env)

    log_path = dataset_root / "nvblox_quickstart.log"
    adapter_log_path = dataset_root / "horus_nvblox_adapter.log"
    launch_command = [
        "ros2",
        "launch",
        "nvblox_examples_bringup",
        "isaac_sim_example.launch.py",
        f"rosbag:={bag}",
        "navigation:=False",
        "run_rviz:=False",
        "run_foxglove:=False",
    ]
    if not args.no_loop:
        launch_command.append("rosbag_args:=--loop")

    nvblox: subprocess.Popen[str] | None = None
    adapter: subprocess.Popen[str] | None = None
    registration_process: subprocess.Popen[str] | None = None
    with (
        log_path.open("w", encoding="utf-8") as log,
        adapter_log_path.open("w", encoding="utf-8") as adapter_log,
    ):
        try:
            print(f"[nvblox] dataset: {bag}")
            print(f"[nvblox] log: {log_path}")
            adapter = subprocess.Popen(
                [
                    sys.executable,
                    str(ADAPTER),
                    "--input-topic",
                    "/nvblox_node/mesh",
                    "--output-topic",
                    "/horus/nvblox/mesh",
                    "--blocks-per-axis",
                    str(args.blocks_per_axis),
                ],
                env=env,
                stdout=adapter_log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            time.sleep(1.0)
            if adapter.poll() is not None:
                raise RuntimeError(f"HORUS nvblox adapter exited with code {adapter.returncode}")
            nvblox = subprocess.Popen(
                launch_command,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            wait_for_mesh_pages(nvblox, adapter, adapter_log_path, args.startup_timeout)
            print("[nvblox] live mesh pages are retained and ready")
            registration = [
                sys.executable,
                str(REGISTRATION),
                "--input-topic",
                "/nvblox_node/mesh",
                "--workspace-scale",
                str(args.workspace_scale),
                "--frame-id",
                args.frame_id,
                "--blocks-per-axis",
                str(args.blocks_per_axis),
                "--no-launch-adapter",
            ]
            registration_process = subprocess.Popen(
                registration,
                env=env,
                text=True,
                start_new_session=True,
            )
            try:
                return registration_process.wait()
            except KeyboardInterrupt:
                return 130
        except RuntimeError:
            if log_path.exists():
                tail = log_path.read_text(errors="replace").splitlines()[-40:]
                if tail:
                    print("\n".join(tail), file=sys.stderr)
            raise
        finally:
            stop_processes(registration_process, nvblox, adapter)


if __name__ == "__main__":
    raise SystemExit(main())
