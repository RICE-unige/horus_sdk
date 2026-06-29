#!/usr/bin/env python3
"""One-command HORUS experiment orchestration."""

from __future__ import annotations

import argparse
import base64
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import time
from typing import Any

SDK_ROOT = Path(__file__).resolve().parents[3]
PYTHON_ROOT = SDK_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from horus.experiments.analysis import metric_row_count, validate_run_directory, write_summary
from horus.experiments.metrics import (
    CsvMetricWriter,
    NdjsonEventWriter,
    clock_anchor_env,
    default_metrics_path,
    now_ns,
    source_metric_stream_seen,
)
from horus.experiments.workloads import load_workload_config


CONTROL_TOPIC = "/horus/experiments/control"
STATUS_TOPIC = "/horus/experiments/status"

ACTIVE_CORE_EXPERIMENT_CONFIGS = {
    "0": "e0_baseline.json",
    "1": "e1_camera_transport.json",
    "3": "e3_pointcloud_capacity.json",
    "4": "e4_mesh_vs_pointcloud_map.json",
    "5": "e5_map_update_behavior.json",
    "6": "e6_control_under_sensor_load.json",
    "7": "e7_multi_robot_scaling.json",
    "8": "e8_multi_operator_scaling.json",
    "9": "e9_failure_recovery.json",
    "10": "e10_camera_capacity_staged.json",
}

RETIRED_CORE_EXPERIMENTS = {
    "2": (
        "E2 used the connector/WebRTC diagnostic path. That transport is out of scope for "
        "the i-RIM core-platform dataset. Continue with E3, or run the legacy config by "
        "explicit name with --allow-connector-transport for a separate connector diagnostic."
    ),
}


def resolve_config(identifier: str) -> Path:
    configs_dir = Path(__file__).resolve().parent / "configs"
    if identifier.isdigit():
        key = str(int(identifier))
        if key in RETIRED_CORE_EXPERIMENTS:
            raise FileNotFoundError(RETIRED_CORE_EXPERIMENTS[key])
        mapped = ACTIVE_CORE_EXPERIMENT_CONFIGS.get(key)
        if mapped:
            return configs_dir / mapped
        raise FileNotFoundError(
            f"No active i-RIM core experiment config found for index {identifier}. "
            "Use an explicit config name for diagnostic or sweep workloads."
        )

    candidate = Path(identifier)
    if candidate.exists():
        return candidate.resolve()

    matches = sorted(configs_dir.glob(f"{identifier}.json"))
    if matches:
        return matches[0]

    matches = sorted(configs_dir.glob(f"{identifier}_*.json"))
    if matches:
        return matches[0]

    raise FileNotFoundError(f"No experiment config found for {identifier}")


def run_setup(config: Path, results_root: Path, run_id: str, notes: str, duration: float, warmup: float) -> Path:
    command = [
        sys.executable,
        str(Path(__file__).resolve().parent / "horus_benchmark_suite.py"),
        "--config",
        str(config),
        "--results-root",
        str(results_root),
    ]
    if run_id:
        command.extend(["--run-id", run_id])
    if notes:
        command.extend(["--notes", notes])
    command.extend(["--duration", str(duration), "--warmup", str(warmup)])
    setup_env = os.environ.copy()
    setup_env["HORUS_SDK_NO_BANNER"] = "1"
    completed = subprocess.run(command, env=setup_env, text=True, capture_output=True, check=True)
    lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    for line in reversed(lines):
        path = Path(line)
        if path.exists():
            return path.resolve()
    raise RuntimeError(f"Could not find run directory in setup output:\n{completed.stdout}")


def experiment_env(run_dir: Path, workload: Any, ros_domain_id: str, ros_localhost_only: str) -> dict[str, str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{PYTHON_ROOT}{os.pathsep}{env.get('PYTHONPATH', '')}".rstrip(os.pathsep)
    env["HORUS_EXPERIMENT_RESULTS_DIR"] = str(run_dir)
    env["HORUS_EXPERIMENT_RUN_ID"] = run_dir.name
    env["HORUS_EXPERIMENT"] = workload.experiment
    env["HORUS_EXPERIMENT_CONDITION"] = workload.condition
    env["HORUS_EXPERIMENT_GATE_HEAVY_STREAMS"] = "1"
    env["ROS_DOMAIN_ID"] = ros_domain_id
    env["ROS_LOCALHOST_ONLY"] = ros_localhost_only
    env["HORUS_SDK_NO_BANNER"] = "1"
    return env


def camera_dimensions(workload: Any) -> tuple[int, int]:
    text = str(getattr(workload.camera, "resolution", "") or "").lower().replace(" ", "")
    if "x" not in text:
        return 1280, 720
    width_text, height_text = text.split("x", 1)
    try:
        width = max(1, int(width_text))
        height = max(1, int(height_text))
    except ValueError:
        return 1280, 720
    return width, height


def camera_fps(workload: Any) -> int:
    try:
        return max(1, int(round(float(workload.camera.fps))))
    except (TypeError, ValueError):
        return 30


def camera_bitrate_kbit(workload: Any) -> int:
    try:
        mbps = float(workload.camera.bitrate_mbps)
    except (TypeError, ValueError):
        mbps = 0.0
    return max(1000, int(round(mbps * 1000.0))) if mbps > 0.0 else 6000


def start_process(command: list[str], env: dict[str, str], cwd: Path | None = None) -> subprocess.Popen:
    kwargs: dict[str, Any] = {
        "env": env,
        "cwd": str(cwd or SDK_ROOT),
        "text": True,
    }
    if os.name != "nt":
        kwargs["preexec_fn"] = os.setsid
    return subprocess.Popen(command, **kwargs)


def start_shell_process(command: str, env: dict[str, str], cwd: Path | None = None) -> subprocess.Popen:
    kwargs: dict[str, Any] = {
        "env": env,
        "cwd": str(cwd or SDK_ROOT),
        "text": True,
        "shell": True,
    }
    if os.name != "nt":
        kwargs["preexec_fn"] = os.setsid
    return subprocess.Popen(command, **kwargs)


def stop_process(process: subprocess.Popen, name: str, timeout: float = 5.0) -> None:
    if process.poll() is not None:
        return
    try:
        if os.name != "nt":
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        else:
            process.terminate()
        process.wait(timeout=timeout)
    except Exception:
        try:
            if os.name != "nt":
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            else:
                process.kill()
        except Exception:
            pass
    finally:
        if process.poll() is None:
            print(f"[experiment] warning: could not stop {name}", file=sys.stderr)


def check_processes_running(
    run_dir: Path,
    processes: list[tuple[str, subprocess.Popen]],
    *,
    phase: str,
    allow_successful_exit: bool = False,
) -> bool:
    failed = []
    completed = []
    for name, process in processes:
        code = process.poll()
        if code == 0 and allow_successful_exit:
            completed.append({"name": name, "returncode": code})
        elif code is not None:
            failed.append({"name": name, "returncode": code})
    if completed:
        write_orchestrator_event(run_dir, "process_completed", {"phase": phase, "completed": completed})
    if not failed:
        return True
    write_orchestrator_event(run_dir, "process_failure", {"phase": phase, "failed": failed})
    for item in failed:
        print(
            f"[experiment] {phase} process {item['name']} exited with {item['returncode']}",
            file=sys.stderr,
        )
    return False


def live_processes(processes: list[tuple[str, subprocess.Popen]]) -> list[tuple[str, subprocess.Popen]]:
    return [(name, process) for name, process in processes if process.poll() is None]


def shell_quote(value: str) -> str:
    return shlex.quote(str(value))


def shell_path(value: str) -> str:
    text = str(value)
    if text == "~":
        return "$HOME"
    if text.startswith("~/"):
        return "$HOME/" + shell_quote(text[2:])
    return shell_quote(text)


def bash_lc(command: str) -> str:
    return "bash -lc " + shell_quote(command)


def connector_env_file(env_vars: dict[str, str]) -> str:
    run_dir = str(env_vars.get("HORUS_RUN_DIR", "")).rstrip("/")
    if not run_dir:
        raise ValueError("Connector launch requires HORUS_RUN_DIR.")
    return str(env_vars.get("HORUS_ENV") or f"{run_dir}/horus_connector.env")


def connector_env_write_command(env_vars: dict[str, str]) -> str:
    env_file = connector_env_file(env_vars)
    lines = [f"{key}={shell_quote(str(value))}" for key, value in sorted(env_vars.items())]
    payload = " ".join(shell_quote(line) for line in lines)
    return f"printf '%s\\n' {payload} > {shell_path(env_file)}"


def local_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        sock.close()


def default_ssh_command() -> str:
    windows_ssh = Path("/mnt/c/Windows/System32/OpenSSH/ssh.exe")
    return str(windows_ssh) if windows_ssh.exists() else "ssh"


def default_adb_command() -> str:
    env_adb = os.getenv("ADB")
    if env_adb:
        return env_adb
    unity_adb = Path("/mnt/c/Program Files/Unity/Hub/Editor/6000.3.8f1/Editor/Data/PlaybackEngines/AndroidPlayer/SDK/platform-tools/adb.exe")
    if unity_adb.exists():
        return shlex.quote(str(unity_adb))
    return "adb"


def ssh_parts(command: str) -> list[str]:
    return shlex.split(command or "ssh")


def ssh_command(args: argparse.Namespace, host: str, remote_command: str) -> list[str]:
    return [*ssh_parts(args.ssh_command), host, remote_command]


def ssh_first_ip(host: str, ssh_command_value: str, timeout: float = 6.0) -> str:
    if not host:
        return ""
    command = [*ssh_parts(ssh_command_value), host, "hostname -I | awk '{print $1}'"]
    try:
        completed = subprocess.run(command, text=True, capture_output=True, timeout=timeout, check=False)
    except Exception:
        return ""
    return completed.stdout.strip().split()[0] if completed.stdout.strip() else ""


def write_orchestrator_event(run_dir: Path, name: str, fields: dict[str, Any]) -> None:
    path = run_dir / "orchestrator_events.ndjson"
    payload = {
        "timestamp_ns": now_ns(),
        "monotonic_ns": time.monotonic_ns(),
        "run_id": run_dir.name,
        "name": name,
        **fields,
    }
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n")


def write_workload_start_signal(run_dir: Path, workload: Any) -> Path:
    path = run_dir / "workload_start.json"
    payload = {
        "action": "workload_start",
        "run_id": run_dir.name,
        "experiment": workload.experiment,
        "condition": workload.condition,
        "timestamp_ns": now_ns(),
        "monotonic_ns": time.monotonic_ns(),
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_orchestrator_event(run_dir, "workload_start_signal_written", {"path": str(path)})
    return path


def write_transport_manifest(run_dir: Path, payload: dict[str, Any]) -> None:
    path = run_dir / "transport_manifest.json"
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def remote_connector_command(
    *,
    connector_root: str,
    role: str,
    topology: str,
    env_vars: dict[str, str],
) -> str:
    run_dir = str(env_vars.get("HORUS_RUN_DIR", ""))
    mkdir = f"mkdir -p {shell_path(run_dir)} && " if run_dir else ""
    env_file = connector_env_file(env_vars)
    command_env = f"HORUS_RUN_DIR={shell_quote(run_dir)} HORUS_ENV={shell_quote(env_file)}"
    return (
        f"{mkdir}{connector_env_write_command({**env_vars, 'HORUS_ENV': env_file})} && "
        f"cd {shell_path(connector_root)} && {command_env} ./horus launch {shell_quote(role)} --no-monitor"
    )


def connector_stop_command(*, connector_root: str, env_vars: dict[str, str]) -> str:
    run_dir = str(env_vars.get("HORUS_RUN_DIR", ""))
    env_file = connector_env_file(env_vars)
    command_env = f"HORUS_RUN_DIR={shell_quote(run_dir)} HORUS_ENV={shell_quote(env_file)}"
    return (
        f"mkdir -p {shell_path(run_dir)} && "
        f"{connector_env_write_command({**env_vars, 'HORUS_ENV': env_file})} && "
        f"cd {shell_path(connector_root)} && {command_env} ./horus stop"
    )


def connector_health_command(run_dir: str, services: list[str]) -> str:
    service_list = " ".join(shell_quote(service) for service in services)
    check = (
        "missing=0; "
        f"for service in {service_list}; do "
        f"pid_file={shell_path(run_dir)}/$service.pid; "
        "if [ ! -f \"$pid_file\" ]; then echo \"missing:$service\"; missing=1; "
        "elif ! kill -0 \"$(cat \"$pid_file\")\" >/dev/null 2>&1; then echo \"stopped:$service\"; missing=1; "
        "fi; "
        "done; "
        "exit $missing"
    )
    return bash_lc(check)


def run_transport_health_check(
    args: argparse.Namespace,
    run_dir: Path,
    checks: list[tuple[str, str | None, str, list[str]]],
) -> bool:
    failures: list[dict[str, Any]] = []
    for name, host, connector_run_dir, services in checks:
        command = connector_health_command(connector_run_dir, services)
        if host:
            completed = subprocess.run(
                ssh_command(args, host, command),
                text=True,
                capture_output=True,
                timeout=12.0,
                check=False,
            )
        else:
            completed = subprocess.run(
                command,
                shell=True,
                text=True,
                capture_output=True,
                timeout=12.0,
                check=False,
            )
        if completed.returncode != 0:
            failures.append(
                {
                    "name": name,
                    "host": host or "local",
                    "run_dir": connector_run_dir,
                    "services": services,
                    "stdout": completed.stdout.strip(),
                    "stderr": completed.stderr.strip(),
                    "returncode": completed.returncode,
                }
            )
    if not failures:
        write_orchestrator_event(run_dir, "transport_health_ok", {"checks": len(checks)})
        return True
    write_orchestrator_event(run_dir, "transport_health_failed", {"failures": failures})
    for failure in failures:
        print(
            "[experiment] transport health failed "
            f"{failure['name']} on {failure['host']}: {failure['stdout'] or failure['stderr']}",
            file=sys.stderr,
        )
    return False


def workload_uses_connector_media(workload: Any) -> bool:
    return "webrtc" in str(workload.transport).lower() or str(workload.camera.encoding).lower() in {"h264", "webrtc"}


def _positive_number(value: Any) -> bool:
    try:
        return float(value) > 0.0
    except (TypeError, ValueError):
        return False


def robot_pointcloud_enabled(workload: Any) -> bool:
    extra = dict(getattr(workload, "extra", {}) or {})
    nested = extra.get("extra")
    if isinstance(nested, dict):
        extra.update(nested)
    value = extra.get("robot_pointcloud", True)
    return value is not False and str(value).strip().lower() not in {"0", "false", "no", "off"}


def expected_source_streams_after_start(workload: Any) -> list[str]:
    streams: list[str] = []
    if _positive_number(getattr(workload.camera, "streams", 0)) and not workload_uses_connector_media(workload):
        streams.append("camera")
    if (
        robot_pointcloud_enabled(workload)
        and _positive_number(getattr(workload.pointcloud, "points", 0))
        and _positive_number(getattr(workload.pointcloud, "hz", 0))
    ):
        streams.append("pointcloud")
    representation = str(getattr(workload.map, "representation", "") or "").lower()
    if "pointcloud" in representation:
        streams.append("map_pointcloud")
    if _positive_number(getattr(workload.map, "triangles", 0)):
        streams.append("mesh")
    return streams


def effective_transport_profile(args: argparse.Namespace, workload: Any) -> str:
    needs_webrtc = workload_uses_connector_media(workload)
    if args.transport_profile == "auto":
        return "local" if needs_webrtc else "none"
    return str(args.transport_profile)


def remote_result_dir(args: argparse.Namespace, run_dir: Path, *parts: str) -> str:
    root = str(args.remote_results_root).rstrip("/")
    suffix = "/".join(part.strip("/") for part in parts if part)
    return f"{root}/{run_dir.name}" + (f"/{suffix}" if suffix else "")


def sdk_relative_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(SDK_ROOT.resolve()).as_posix()
    except ValueError as exc:
        raise ValueError(
            f"Remote workload configs must live under the SDK root. Got {path} outside {SDK_ROOT}."
        ) from exc


def remote_workload_command(
    args: argparse.Namespace,
    config: Path,
    run_dir: Path,
    workload: Any,
    duration: float,
    env: dict[str, str],
) -> str:
    remote_run_dir = remote_result_dir(args, run_dir)
    setup = [
        f"mkdir -p {shell_path(remote_run_dir)}",
        f"cd {shell_path(args.remote_sdk_root)}",
        "source /opt/ros/jazzy/setup.bash",
        f"export PYTHONPATH=python:${{PYTHONPATH:-}}",
        f"export ROS_DOMAIN_ID={shell_quote(env['ROS_DOMAIN_ID'])}",
        f"export ROS_LOCALHOST_ONLY={shell_quote(env['ROS_LOCALHOST_ONLY'])}",
        f"export HORUS_EXPERIMENT_RESULTS_DIR={shell_path(remote_run_dir)}",
        f"export HORUS_EXPERIMENT_RUN_ID={shell_quote(run_dir.name)}",
        f"export HORUS_EXPERIMENT={shell_quote(workload.experiment)}",
        f"export HORUS_EXPERIMENT_CONDITION={shell_quote(workload.condition)}",
    ]
    if args.workload_command:
        command = args.workload_command
    else:
        relative_config = sdk_relative_path(config)
        command = (
            "python3 python/examples/experiments/horus_synthetic_workload.py "
            f"--config {shell_quote(relative_config)} "
            f"--duration {shell_quote(str(duration))} "
            f"--metrics-csv {shell_path(remote_run_dir + '/source_metrics.csv')}"
        )
    return bash_lc(" && ".join(setup + [command]))


def copy_remote_workload_results(args: argparse.Namespace, run_dir: Path) -> None:
    if not args.workload_host or args.no_copy_remote_workload_results:
        return
    remote_run_dir = remote_result_dir(args, run_dir)
    copied: list[str] = []
    missing: list[str] = []
    for name in ("source_metrics.csv", "source_events.ndjson", "operator_metrics.csv", "failure_metrics.csv"):
        target = run_dir / name
        remote_file = f"{remote_run_dir}/{name}"
        completed = subprocess.run(
            ssh_command(args, args.workload_host, f"cat {shell_path(remote_file)}"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if completed.returncode == 0:
            target.write_bytes(completed.stdout)
            copied.append(name)
        else:
            missing.append(name)
    write_orchestrator_event(
        run_dir,
        "remote_workload_results_copied",
        {"host": args.workload_host, "remote_run_dir": remote_run_dir, "copied": copied, "missing": missing},
    )


def stop_transport_services(args: argparse.Namespace, workload: Any, run_dir: Path, env: dict[str, str]) -> None:
    if args.no_transport:
        return
    profile = effective_transport_profile(args, workload)
    if profile == "none":
        return

    machine_ip = args.machine_ip if args.machine_ip != "auto" else local_ip()
    cloud_ip = args.cloud_ip if args.cloud_ip != "auto" else args.cloud_host
    common = {
        "ROS_DOMAIN_ID": env["ROS_DOMAIN_ID"],
        "ROS_LOCALHOST_ONLY": env["ROS_LOCALHOST_ONLY"],
    }
    cleanup: list[tuple[str, list[str] | str, Path | None]] = []

    def add_local(name: str, role: str, topology: str, run_dir_value: str, extra: dict[str, str] | None = None) -> None:
        command = connector_stop_command(
            connector_root=args.local_connector_root,
            env_vars={**common, "HORUS_RUN_DIR": run_dir_value, "HORUS_ROLE": role, "HORUS_TOPOLOGY": topology, **(extra or {})},
        )
        cleanup.append((name, command, Path(args.local_connector_root).expanduser()))

    def add_remote(host: str, name: str, role: str, topology: str, run_dir_value: str, extra: dict[str, str] | None = None) -> None:
        command = connector_stop_command(
            connector_root=args.connector_root,
            env_vars={**common, "HORUS_RUN_DIR": run_dir_value, "HORUS_ROLE": role, "HORUS_TOPOLOGY": topology, **(extra or {})},
        )
        cleanup.append((name, ssh_command(args, host, command), None))

    if profile == "local":
        add_local("connector_machine_stop", "machine", "direct", str(run_dir / "connector" / "machine"), {"HORUS_MACHINE_IP": machine_ip})
        add_local("connector_robot_stop", "robot", "direct", str(run_dir / "connector" / "robot"), {"HORUS_MACHINE_IP": machine_ip})
    elif profile == "direct":
        add_local("connector_machine_stop", "machine", "direct", str(run_dir / "connector" / "machine"), {"HORUS_MACHINE_IP": machine_ip})
        add_remote(args.robot_host, "connector_robot_stop", "robot", "direct", remote_result_dir(args, run_dir, "connector", "robot"), {"HORUS_MACHINE_IP": machine_ip})
    elif profile == "cloud":
        add_local("connector_machine_stop", "machine", "hub", str(run_dir / "connector" / "machine"), {"HORUS_CLOUD_IP": cloud_ip})
        add_remote(args.robot_host, "connector_robot_stop", "robot", "hub", remote_result_dir(args, run_dir, "connector", "robot"), {"HORUS_CLOUD_IP": cloud_ip})
        add_remote(args.cloud_host, "connector_cloud_stop", "cloud", "hub", remote_result_dir(args, run_dir, "connector", "cloud"), {"HORUS_CLOUD_IP": cloud_ip})

    for name, command, cwd in cleanup:
        try:
            if isinstance(command, str):
                completed = subprocess.run(command, env=env, cwd=str(cwd or SDK_ROOT), shell=True, text=True, timeout=20.0, check=False)
            else:
                completed = subprocess.run(command, env=env, cwd=str(cwd or SDK_ROOT), text=True, timeout=20.0, check=False)
            write_orchestrator_event(run_dir, "transport_cleanup", {"name": name, "returncode": completed.returncode})
        except Exception as exc:
            write_orchestrator_event(run_dir, "transport_cleanup_failed", {"name": name, "error": str(exc)})
            print(f"[experiment] warning: transport cleanup failed for {name}: {exc}", file=sys.stderr)


def maybe_start_transport_processes(
    args: argparse.Namespace,
    workload: Any,
    run_dir: Path,
    env: dict[str, str],
) -> list[tuple[str, subprocess.Popen]]:
    if args.no_transport:
        write_transport_manifest(run_dir, {"enabled": False, "reason": "disabled_by_argument"})
        return []

    needs_webrtc = workload_uses_connector_media(workload)
    profile = effective_transport_profile(args, workload)
    if profile == "none":
        write_transport_manifest(run_dir, {"enabled": False, "reason": "profile_none", "needs_webrtc": needs_webrtc})
        return []

    streams = max(1, int(workload.camera.streams or 1))
    first_input_topic = "/exp_robot_0/camera_0/image_raw"
    first_output_topic = "/exp_robot_0/camera_0/webrtc/image_raw"
    width, height = camera_dimensions(workload)
    fps = camera_fps(workload)
    bitrate_kbit = camera_bitrate_kbit(workload)
    machine_ip = args.machine_ip if args.machine_ip != "auto" else local_ip()
    cloud_ip = args.cloud_ip
    if cloud_ip == "auto":
        cloud_ip = ssh_first_ip(args.cloud_host, args.ssh_command) or args.cloud_host

    manifest: dict[str, Any] = {
        "enabled": True,
        "profile": profile,
        "streams_requested": streams,
        "streams_launched": 1,
        "local_zenoh_enabled": False if profile == "local" else True,
        "input_topic": first_input_topic,
        "output_topic": first_output_topic,
        "machine_ip": machine_ip,
        "cloud_host": args.cloud_host,
        "cloud_ip": cloud_ip,
        "robot_host": args.robot_host,
        "workload_host": args.workload_host,
        "notes": "The local WebRTC profile is media-only and disables Zenoh; it launches one immersive camera stream.",
    }
    processes: list[tuple[str, subprocess.Popen]] = []

    common = {
        "ROS_DOMAIN_ID": env["ROS_DOMAIN_ID"],
        "ROS_LOCALHOST_ONLY": env["ROS_LOCALHOST_ONLY"],
        "HORUS_ROOM": run_dir.name,
        "WEBRTC_MEDIA_MODE": "h264",
        "WEBRTC_ROS_IMAGE_INPUT_TOPIC": first_input_topic,
        "WEBRTC_ROS_IMAGE_OUTPUT_TOPIC": first_output_topic,
        "WEBRTC_ROS_IMAGE_TOPIC": first_input_topic,
        "WEBRTC_VIDEO_WIDTH": str(width),
        "WEBRTC_VIDEO_HEIGHT": str(height),
        "WEBRTC_VIDEO_FPS": str(fps),
        "VIDEO_BITRATE_KBIT": str(bitrate_kbit),
        "WEBRTC_MAX_BITRATE_KBIT": str(bitrate_kbit),
        "WEBRTC_LATENCY_PROBE": "1",
        "WEBRTC_LATENCY_PROBE_RATE": str(min(60, fps)),
        "WEBRTC_FRESH_DEADLINE_MS": "150.0",
    }

    if profile == "cloud":
        cloud_run_dir = remote_result_dir(args, run_dir, "connector", "cloud")
        robot_run_dir = remote_result_dir(args, run_dir, "connector", "robot")
        cloud_cmd = remote_connector_command(
            connector_root=args.connector_root,
            role="cloud",
            topology="hub",
            env_vars={**common, "HORUS_RUN_DIR": cloud_run_dir, "HORUS_ROLE": "cloud", "HORUS_TOPOLOGY": "hub", "HORUS_CLOUD_IP": cloud_ip},
        )
        robot_cmd = remote_connector_command(
            connector_root=args.connector_root,
            role="robot",
            topology="hub",
            env_vars={**common, "HORUS_RUN_DIR": robot_run_dir, "HORUS_ROLE": "robot", "HORUS_TOPOLOGY": "hub", "HORUS_CLOUD_IP": cloud_ip},
        )
        machine_cmd = remote_connector_command(
            connector_root=args.local_connector_root,
            role="machine",
            topology="hub",
            env_vars={
                **common,
                "HORUS_RUN_DIR": str(run_dir / "connector" / "machine"),
                "HORUS_ROLE": "machine",
                "HORUS_TOPOLOGY": "hub",
                "HORUS_CLOUD_IP": cloud_ip,
                "WEBRTC_LATENCY_JSON": str(run_dir / "connector" / "machine" / "webrtc_latency.json"),
            },
        )
        processes.append(("connector_cloud", start_process(ssh_command(args, args.cloud_host, cloud_cmd), env)))
        time.sleep(max(0.0, args.transport_role_startup_gap))
        processes.append(("connector_robot", start_process(ssh_command(args, args.robot_host, robot_cmd), env)))
        processes.append(("connector_machine", start_shell_process(machine_cmd, env, cwd=Path(args.local_connector_root).expanduser())))
    elif profile == "direct":
        robot_run_dir = remote_result_dir(args, run_dir, "connector", "robot")
        robot_cmd = remote_connector_command(
            connector_root=args.connector_root,
            role="robot",
            topology="direct",
            env_vars={**common, "HORUS_RUN_DIR": robot_run_dir, "HORUS_ROLE": "robot", "HORUS_TOPOLOGY": "direct", "HORUS_MACHINE_IP": machine_ip},
        )
        machine_cmd = remote_connector_command(
            connector_root=args.local_connector_root,
            role="machine",
            topology="direct",
            env_vars={
                **common,
                "HORUS_RUN_DIR": str(run_dir / "connector" / "machine"),
                "HORUS_ROLE": "machine",
                "HORUS_TOPOLOGY": "direct",
                "HORUS_MACHINE_IP": machine_ip,
                "WEBRTC_LATENCY_JSON": str(run_dir / "connector" / "machine" / "webrtc_latency.json"),
            },
        )
        processes.append(("connector_machine", start_shell_process(machine_cmd, env, cwd=Path(args.local_connector_root).expanduser())))
        time.sleep(max(0.0, args.transport_role_startup_gap))
        processes.append(("connector_robot", start_process(ssh_command(args, args.robot_host, robot_cmd), env)))
    elif profile == "local":
        local_media_only = {
            "HORUS_ZENOH_ENABLED": "0",
            "HORUS_WEBRTC_SIGNAL_IP": machine_ip,
        }
        robot_cmd = remote_connector_command(
            connector_root=args.local_connector_root,
            role="robot",
            topology="direct",
            env_vars={
                **common,
                **local_media_only,
                "HORUS_RUN_DIR": str(run_dir / "connector" / "robot"),
                "HORUS_ROLE": "robot",
                "HORUS_TOPOLOGY": "direct",
                "HORUS_MACHINE_IP": machine_ip,
            },
        )
        machine_cmd = remote_connector_command(
            connector_root=args.local_connector_root,
            role="machine",
            topology="direct",
            env_vars={
                **common,
                **local_media_only,
                "HORUS_RUN_DIR": str(run_dir / "connector" / "machine"),
                "HORUS_ROLE": "machine",
                "HORUS_TOPOLOGY": "direct",
                "HORUS_MACHINE_IP": machine_ip,
                "WEBRTC_LATENCY_JSON": str(run_dir / "connector" / "machine" / "webrtc_latency.json"),
            },
        )
        processes.append(("connector_machine", start_shell_process(machine_cmd, env, cwd=Path(args.local_connector_root).expanduser())))
        time.sleep(max(0.0, args.transport_role_startup_gap))
        processes.append(("connector_robot", start_shell_process(robot_cmd, env, cwd=Path(args.local_connector_root).expanduser())))
    else:
        raise ValueError(f"Unknown transport profile: {profile}")

    for index, command in enumerate(args.remote_command or []):
        name = f"remote_{index}"
        if "=" in command.split(" ", 1)[0]:
            maybe_name, command = command.split("=", 1)
            name = maybe_name.strip() or name
        processes.append((name, start_shell_process(command, env)))

    write_transport_manifest(run_dir, manifest)
    write_orchestrator_event(run_dir, "transport_started", manifest)
    return processes


def transport_health_checks(
    args: argparse.Namespace,
    workload: Any,
    run_dir: Path,
) -> list[tuple[str, str | None, str, list[str]]]:
    profile = effective_transport_profile(args, workload)
    if args.no_transport or profile == "none":
        return []
    if profile == "local":
        return [
            ("connector_machine", None, str(run_dir / "connector" / "machine"), ["webrtc"]),
            ("connector_robot", None, str(run_dir / "connector" / "robot"), ["webrtc"]),
        ]
    if profile == "direct":
        return [
            ("connector_machine", None, str(run_dir / "connector" / "machine"), ["zenoh", "webrtc"]),
            ("connector_robot", args.robot_host, remote_result_dir(args, run_dir, "connector", "robot"), ["zenoh", "webrtc"]),
        ]
    if profile == "cloud":
        return [
            ("connector_cloud", args.cloud_host, remote_result_dir(args, run_dir, "connector", "cloud"), ["zenoh", "signal"]),
            ("connector_machine", None, str(run_dir / "connector" / "machine"), ["zenoh", "webrtc"]),
            ("connector_robot", args.robot_host, remote_result_dir(args, run_dir, "connector", "robot"), ["zenoh", "webrtc"]),
        ]
    return []


def collect_webrtc_metrics(run_dir: Path, workload: Any) -> None:
    latency_path = run_dir / "connector" / "machine" / "webrtc_latency.json"
    if not latency_path.exists():
        return
    try:
        payload = json.loads(latency_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        write_orchestrator_event(run_dir, "webrtc_metrics_parse_failed", {"path": str(latency_path), "error": str(exc)})
        return

    width, height = camera_dimensions(workload)
    topic = "/exp_robot_0/camera_0/webrtc/image_raw"
    fieldnames = (
        "monotonic_ns",
        "source",
        "category",
        "name",
        "robot_id",
        "stream",
        "topic",
        "seq",
        "payload_bytes",
        "duration_ms",
        "value",
        "unit",
        "queue_depth",
        "frame_width",
        "frame_height",
        "points",
        "vertices",
        "triangles",
        "extra_json",
    )
    rows_written = 0
    with CsvMetricWriter(
        run_dir / "webrtc_metrics.csv",
        run_id=run_dir.name,
        experiment=workload.experiment,
        condition=workload.condition,
        fieldnames=fieldnames,
    ) as writer:
        summary_metrics = {
            "video_decoded_fps": "fps",
            "video_frames": "frames",
            "video_observed_sec": "s",
            "video_latency_samples": "samples",
            "video_latency_ms_median": "ms",
            "video_latency_ms_p95": "ms",
            "video_latency_ms_p99": "ms",
            "fresh_sample_sla": "ratio",
            "fresh_fps_estimate": "fps",
        }
        for name, unit in summary_metrics.items():
            value = payload.get(name)
            if isinstance(value, (int, float)):
                writer.write(
                    {
                        "monotonic_ns": time.monotonic_ns(),
                        "source": "horus_connector",
                        "category": "webrtc",
                        "name": name,
                        "robot_id": "exp_robot_0",
                        "stream": "camera_0",
                        "topic": topic,
                        "value": value,
                        "unit": unit,
                        "frame_width": width,
                        "frame_height": height,
                        "extra_json": json.dumps({"latency_json": str(latency_path)}, separators=(",", ":")),
                    }
                )
                rows_written += 1
        for sample in payload.get("samples", []):
            if not isinstance(sample, dict):
                continue
            latency = sample.get("latency_ms")
            if not isinstance(latency, (int, float)):
                continue
            writer.write(
                {
                    "monotonic_ns": time.monotonic_ns(),
                    "source": "horus_connector",
                    "category": "webrtc",
                    "name": "frame_latency_ms",
                    "robot_id": "exp_robot_0",
                    "stream": "camera_0",
                    "topic": topic,
                    "seq": sample.get("seq") or 0,
                    "duration_ms": latency,
                    "value": latency,
                    "unit": "ms",
                    "frame_width": width,
                    "frame_height": height,
                    "extra_json": json.dumps({"t_sec": sample.get("t_sec"), "raw_latency_ms": sample.get("raw_latency_ms")}, separators=(",", ":")),
                }
            )
            rows_written += 1
    write_orchestrator_event(run_dir, "webrtc_metrics_collected", {"path": str(latency_path), "rows": rows_written})


def local_headset_export_name(file_name: str) -> str:
    if file_name == "headset_metrics.csv":
        return file_name
    if file_name == "events.ndjson":
        return "headset_events.ndjson"
    if file_name == "run_manifest.json":
        return "headset_run_manifest.json"
    if file_name.endswith("_metrics.csv"):
        return f"headset_{file_name}"
    return f"headset_{Path(file_name).name}"


def pull_headset_export_with_adb(run_dir: Path, adb_command: str, quest_package: str) -> bool:
    if not adb_command:
        return False

    remote_root = f"/storage/emulated/0/Android/data/{quest_package}/files/Horus/Experiments/{run_dir.name}"
    files = {
        "headset_metrics.csv": "headset_metrics.csv",
        "events.ndjson": "headset_events.ndjson",
        "run_manifest.json": "headset_run_manifest.json",
    }
    pulled: list[str] = []
    try:
        devices = subprocess.run(
            [*shlex.split(adb_command), "devices"],
            text=True,
            capture_output=True,
            timeout=10.0,
            check=False,
        )
    except Exception as exc:
        write_orchestrator_event(run_dir, "headset_adb_export_failed", {"error": f"adb unavailable: {exc}"})
        return False

    device_lines = [line for line in devices.stdout.splitlines() if line.strip().endswith("\tdevice")]
    if not device_lines:
        write_orchestrator_event(run_dir, "headset_adb_export_skipped", {"reason": "no_adb_device"})
        return False

    for remote_name, local_name in files.items():
        try:
            result = subprocess.run(
                [*shlex.split(adb_command), "pull", f"{remote_root}/{remote_name}", str(run_dir / local_name)],
                text=True,
                capture_output=True,
                timeout=30.0,
                check=False,
            )
        except Exception as exc:
            write_orchestrator_event(run_dir, "headset_adb_export_file_failed", {"file": remote_name, "error": str(exc)})
            continue
        if result.returncode == 0:
            pulled.append(local_name)
        else:
            write_orchestrator_event(
                run_dir,
                "headset_adb_export_file_failed",
                {"file": remote_name, "returncode": result.returncode, "stderr": result.stderr.strip()},
            )

    write_orchestrator_event(run_dir, "headset_adb_export_received", {"files": pulled})
    return "headset_metrics.csv" in pulled


def collect_headset_export(
    control: "ExperimentControlNode",
    run_dir: Path,
    timeout_s: float,
    *,
    adb_command: str = "",
    quest_package: str = "",
) -> bool:
    request_id = f"{run_dir.name}-{now_ns()}"
    chunks: dict[str, dict[int, str]] = {}
    chunk_counts: dict[str, int] = {}
    completed = False
    error = ""
    start_index = len(control.status_messages)

    control.send_action("export", repeats=1, run_id=run_dir.name, request_id=request_id)
    deadline = time.monotonic() + max(1.0, timeout_s)
    cursor = start_index
    while time.monotonic() < deadline and not completed and not error:
        rclpy.spin_once(control, timeout_sec=0.1)
        while cursor < len(control.status_messages):
            message = control.status_messages[cursor]
            cursor += 1
            if message.get("request_id") != request_id:
                continue
            status = str(message.get("status") or "")
            if status == "export_error":
                error = str(message.get("message") or "unknown export error")
                break
            if status == "export_chunk":
                file_name = str(message.get("file_name") or "")
                if not file_name:
                    continue
                try:
                    index = int(message.get("chunk_index"))
                    count = int(message.get("chunk_count"))
                except (TypeError, ValueError):
                    error = f"invalid chunk metadata for {file_name}"
                    break
                chunks.setdefault(file_name, {})[index] = str(message.get("data") or "")
                chunk_counts[file_name] = count
            elif status == "export_end":
                completed = True
                break

    if error:
        write_orchestrator_event(run_dir, "headset_export_failed", {"request_id": request_id, "error": error})
        return pull_headset_export_with_adb(run_dir, adb_command, quest_package)
    if not completed:
        write_orchestrator_event(run_dir, "headset_export_timeout", {"request_id": request_id, "timeout_s": timeout_s})
        return pull_headset_export_with_adb(run_dir, adb_command, quest_package)

    copied: list[str] = []
    for file_name, file_chunks in sorted(chunks.items()):
        expected_count = chunk_counts.get(file_name, 0)
        if expected_count <= 0:
            write_orchestrator_event(run_dir, "headset_export_file_skipped", {"file": file_name, "reason": "missing_chunk_count"})
            continue
        missing = [index for index in range(expected_count) if index not in file_chunks]
        if missing:
            write_orchestrator_event(
                run_dir,
                "headset_export_file_skipped",
                {"file": file_name, "reason": "missing_chunks", "missing": missing[:20], "missing_count": len(missing)},
            )
            continue
        encoded = "".join(file_chunks[index] for index in range(expected_count))
        try:
            data = base64.b64decode(encoded.encode("ascii"), validate=True)
        except Exception as exc:
            write_orchestrator_event(run_dir, "headset_export_file_skipped", {"file": file_name, "reason": f"decode_failed: {exc}"})
            continue
        target = run_dir / local_headset_export_name(file_name)
        target.write_bytes(data)
        copied.append(target.name)

    write_orchestrator_event(run_dir, "headset_export_received", {"request_id": request_id, "files": copied})
    if "headset_metrics.csv" in copied:
        return True
    return pull_headset_export_with_adb(run_dir, adb_command, quest_package)


class ExperimentControlNode(Node):
    def __init__(self) -> None:
        super().__init__("horus_experiment_orchestrator")
        self.publisher = self.create_publisher(String, CONTROL_TOPIC, 10)
        self.create_subscription(String, STATUS_TOPIC, self.on_status, 1000)
        self.status_messages: list[dict[str, Any]] = []

    def on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                payload["_host_receive_ns"] = now_ns()
                payload["_host_receive_monotonic_ns"] = time.monotonic_ns()
            self.status_messages.append(payload)
        except json.JSONDecodeError:
            pass

    def send_action(self, action: str, repeats: int = 3, **fields: Any) -> None:
        payload = {"action": action, **fields}
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        repeat_count = max(1, repeats)
        for index in range(repeat_count):
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            if index < repeat_count - 1:
                time.sleep(0.15)
        print(f"[experiment] sent {action}: {msg.data}")


def _status_matches_run(message: dict[str, Any], run_id: str) -> bool:
    message_run_id = str(message.get("run_id") or "")
    return not message_run_id or message_run_id == run_id


def wait_for_headset_status(
    control: ExperimentControlNode,
    run_dir: Path,
    accepted_statuses: set[str],
    *,
    timeout_s: float,
    run_id: str = "",
    start_index: int = 0,
) -> tuple[bool, int, dict[str, Any]]:
    deadline = time.monotonic() + max(0.0, timeout_s)
    cursor = max(0, start_index)
    last_message: dict[str, Any] = {}
    while time.monotonic() < deadline:
        rclpy.spin_once(control, timeout_sec=0.1)
        while cursor < len(control.status_messages):
            message = control.status_messages[cursor]
            cursor += 1
            last_message = message
            write_orchestrator_event(run_dir, "headset_status", {"message": message})
            status = str(message.get("status") or "")
            if status in accepted_statuses and (not run_id or _status_matches_run(message, run_id)):
                return True, cursor, message
        time.sleep(0.05)
    return False, cursor, last_message


def send_action_until_status(
    control: ExperimentControlNode,
    run_dir: Path,
    action: str,
    accepted_statuses: set[str],
    *,
    timeout_s: float,
    publish_interval_s: float = 0.75,
    include_existing_status: bool = False,
    **fields: Any,
) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_s)
    cursor = 0 if include_existing_status else len(control.status_messages)
    run_id = str(fields.get("run_id") or "")
    last_publish = 0.0
    last_message: dict[str, Any] = {}
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_publish >= publish_interval_s:
            control.send_action(action, repeats=1, **fields)
            last_publish = now
        ok, cursor, last_message = wait_for_headset_status(
            control,
            run_dir,
            accepted_statuses,
            timeout_s=min(0.25, max(0.0, deadline - time.monotonic())),
            run_id=run_id,
            start_index=cursor,
        )
        if ok:
            write_orchestrator_event(run_dir, f"headset_{action}_ack", {"status": last_message})
            return True
    write_orchestrator_event(
        run_dir,
        f"headset_{action}_timeout",
        {"accepted_statuses": sorted(accepted_statuses), "timeout_s": timeout_s, "last_status": last_message},
    )
    return False


def _int_field(payload: dict[str, Any], key: str) -> int:
    value = payload.get(key)
    if isinstance(value, bool):
        raise ValueError(f"{key} is not an integer")
    return int(value)


def _load_clock_sync(run_dir: Path) -> dict[str, Any]:
    path = run_dir / "clock_sync.json"
    if not path.exists():
        return {"samples": []}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {"samples": []}
    if not isinstance(payload, dict):
        return {"samples": []}
    if not isinstance(payload.get("samples"), list):
        payload["samples"] = []
    return payload


def _persist_cli_invocation(run_dir: Path) -> None:
    """Record the exact command line into run_manifest.json so a run is reproducible verbatim,
    even when CLI flags override the committed config."""
    manifest_path = run_dir / "run_manifest.json"
    if not manifest_path.exists():
        return
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return
    if not isinstance(manifest, dict):
        return
    manifest["cli_argv"] = list(sys.argv)
    manifest["cli_invocation"] = " ".join(shlex.quote(arg) for arg in sys.argv)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _annotate_source_clock(run_dir: Path, args: argparse.Namespace) -> bool:
    """Record whether the source (ROS header) clock is synchronized with the host clock that the
    headset offset is computed against, and return True iff it is.

    End-to-end latency = corrected_headset_ts (host clock) - source_stamp_ns (ROS header clock).
    That subtraction is only valid when the publisher shares the host clock. It does when the
    synthetic workload runs co-located on the SDK/WSL host (no --workload-host): the workload and
    orchestrator both use horus.experiments.metrics.now_ns(), which is a wall-time anchor advanced
    by a monotonic clock. When the workload is pushed to a remote robot host (--workload-host
    arancino) there is NO host<->robot handshake, so source_stamp_ns lives in the robot clock and
    the latency carries an uncorrected, load-uncorrelated offset. We refuse to silently report that
    as glass-to-glass: stamp the manifest and warn loudly so the number is either trusted
    (co-located) or excluded (remote)."""
    manifest_path = run_dir / "run_manifest.json"
    if not manifest_path.exists():
        return False
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    if not isinstance(manifest, dict):
        return False

    workload_host = (getattr(args, "workload_host", "") or "").strip()
    custom_command = bool(getattr(args, "workload_command", ""))
    co_located = not workload_host and not custom_command
    synchronized = co_located  # no source<->host handshake exists yet; only co-located is sound
    manifest["source_clock"] = {
        "synchronized": synchronized,
        "co_located_with_host": co_located,
        "workload_host": workload_host or "localhost",
        "custom_workload_command": custom_command,
        "method": "shared_host_clock" if co_located else "none",
        "note": (
            "Source publisher shares the host clock; end-to-end latency is glass-to-glass."
            if co_located
            else "Source publisher runs on a remote host with NO host<->robot clock handshake; "
            "end-to-end latency is NOT glass-to-glass and must be excluded or corrected."
        ),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if not synchronized:
        print(
            "WARNING: source clock is UNSYNCHRONIZED (workload on "
            f"{workload_host or 'custom command'}, no host<->robot handshake). End-to-end latency "
            "will conflate an uncorrected source-clock offset and is NOT glass-to-glass. Either run "
            "the workload co-located on the SDK host, or treat latency from this run as invalid.",
            file=sys.stderr,
        )
    return synchronized


def _merge_clock_sync_summary_into_manifest(run_dir: Path, summary: dict[str, Any]) -> None:
    """Persist the clock-offset summary (offset / RTT / drift / uncertainty) into run_manifest.json
    so a reader of the canonical manifest can bound the latency uncertainty and reproduce the run.
    The offset-uncertainty band is half the minimum round-trip (the classic NTP bound)."""
    manifest_path = run_dir / "run_manifest.json"
    if not manifest_path.exists():
        return
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return
    if not isinstance(manifest, dict):
        return
    clock = dict(summary)
    rtt_min = summary.get("round_trip_ns_min")
    if isinstance(rtt_min, int):
        clock["offset_uncertainty_ns"] = rtt_min // 2
    manifest["clock_sync"] = clock
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_clock_sync(run_dir: Path, sample: dict[str, Any]) -> None:
    payload = _load_clock_sync(run_dir)
    samples = payload.setdefault("samples", [])
    assert isinstance(samples, list)
    samples.append(sample)
    valid_offsets = [
        int(item["offset_ns"])
        for item in samples
        if isinstance(item, dict) and "offset_ns" in item
    ]
    valid_rtts = [
        int(item["round_trip_ns"])
        for item in samples
        if isinstance(item, dict) and "round_trip_ns" in item
    ]
    summary: dict[str, Any] = {"sample_count": len(samples)}
    if valid_offsets:
        summary["offset_ns_min"] = min(valid_offsets)
        summary["offset_ns_max"] = max(valid_offsets)
        summary["offset_ns_last"] = valid_offsets[-1]
        summary["offset_drift_ns"] = valid_offsets[-1] - valid_offsets[0]
    if valid_rtts:
        summary["round_trip_ns_min"] = min(valid_rtts)
        summary["round_trip_ns_max"] = max(valid_rtts)
    payload["summary"] = summary
    (run_dir / "clock_sync.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _merge_clock_sync_summary_into_manifest(run_dir, summary)


def _optional_int_field(message: dict[str, Any], key: str) -> int | None:
    value = message.get(key)
    if value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _collect_one_clock_ping(
    control: ExperimentControlNode,
    run_dir: Path,
    label: str,
    suffix: int,
    timeout_s: float,
) -> dict[str, Any] | None:
    request_id = f"{run_dir.name}-{label}-{suffix}-{now_ns()}"
    cursor = len(control.status_messages)
    host_send_ns = now_ns()
    control.send_action(
        "clock_ping",
        repeats=1,
        run_id=run_dir.name,
        request_id=request_id,
        host_send_ns=host_send_ns,
    )
    deadline = time.monotonic() + max(0.25, timeout_s)
    while time.monotonic() < deadline:
        rclpy.spin_once(control, timeout_sec=0.1)
        while cursor < len(control.status_messages):
            message = control.status_messages[cursor]
            cursor += 1
            if message.get("request_id") != request_id:
                continue
            if str(message.get("status") or "") != "clock_pong":
                continue
            host_receive_ns = _optional_int_field(message, "_host_receive_ns") or now_ns()
            try:
                headset_receive_ns = _int_field(message, "headset_receive_ns")
                headset_send_ns = _int_field(message, "headset_send_ns")
            except (TypeError, ValueError):
                return None

            round_trip_ns = max(0, (host_receive_ns - host_send_ns) - (headset_send_ns - headset_receive_ns))
            offset_ns = int(((headset_receive_ns - host_send_ns) + (headset_send_ns - host_receive_ns)) / 2)
            return {
                "label": label,
                "request_id": request_id,
                "host_send_ns": host_send_ns,
                "host_receive_ns": host_receive_ns,
                "headset_receive_ns": headset_receive_ns,
                "headset_send_ns": headset_send_ns,
                # The simultaneous (UnixTime, Stopwatch) anchor lets analysis map a Quest monotonic_ns
                # interval onto the host timeline (B1.e).
                "headset_receive_monotonic_ns": _optional_int_field(message, "headset_receive_monotonic_ns"),
                "headset_send_monotonic_ns": _optional_int_field(message, "headset_send_monotonic_ns"),
                "round_trip_ns": round_trip_ns,
                "offset_ns": offset_ns,
                "message": message,
            }
        time.sleep(0.02)
    return None


def collect_clock_sync_sample(
    control: ExperimentControlNode,
    run_dir: Path,
    label: str,
    timeout_s: float = 5.0,
    ping_count: int = 11,
) -> dict[str, Any] | None:
    # NTP-style min-RTT handshake: fire a burst of pings and keep the sample with the LOWEST
    # round-trip. A single ping is biased by any one scheduling/Wi-Fi spike; the minimum-RTT
    # exchange carries the least queueing noise, so its offset is the most accurate estimate.
    pings = max(1, ping_count)
    per_ping_timeout = max(0.25, timeout_s / pings) if pings > 1 else timeout_s
    samples: list[dict[str, Any]] = []
    for index in range(pings):
        sample = _collect_one_clock_ping(control, run_dir, label, index, per_ping_timeout)
        if sample is not None:
            samples.append(sample)
        time.sleep(0.01)

    if not samples:
        write_orchestrator_event(
            run_dir,
            "clock_sync_timeout",
            {"label": label, "timeout_s": timeout_s, "ping_count": pings},
        )
        return None

    best = dict(min(samples, key=lambda item: item["round_trip_ns"]))
    best["ping_count"] = len(samples)
    best["round_trip_ns_min_burst"] = min(item["round_trip_ns"] for item in samples)
    best["round_trip_ns_max_burst"] = max(item["round_trip_ns"] for item in samples)
    _write_clock_sync(run_dir, best)
    write_orchestrator_event(
        run_dir,
        "clock_sync_sample",
        {
            "label": label,
            "request_id": best["request_id"],
            "round_trip_ms": best["round_trip_ns"] / 1_000_000.0,
            "offset_ms": best["offset_ns"] / 1_000_000.0,
            "pings_received": len(samples),
            "pings_requested": pings,
        },
    )
    return best


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", help="Experiment number, e.g. 0, or a config path/name.")
    parser.add_argument("--results-root", default=str(SDK_ROOT / "results"))
    parser.add_argument("--run-id", default="")
    parser.add_argument("--notes", default="")
    parser.add_argument("--ros-domain-id", default=os.getenv("ROS_DOMAIN_ID", "51"))
    parser.add_argument("--ros-localhost-only", default=os.getenv("ROS_LOCALHOST_ONLY", "1"))
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--warmup", type=float, default=-1.0)
    parser.add_argument("--no-bridge", action="store_true")
    parser.add_argument("--bridge-command", default="ros2 launch horus_unity_bridge unity_bridge.launch.py enable_priority_scheduling:=true")
    parser.add_argument("--bridge-wait", type=float, default=5.0)
    parser.add_argument("--transport-startup-wait", type=float, default=2.0)
    parser.add_argument("--ready-mode", choices=("prompt", "delay", "none"), default="prompt")
    parser.add_argument("--ready-delay", type=float, default=0.0)
    parser.add_argument("--no-registration", action="store_true")
    parser.add_argument("--force-registration", action="store_true")
    parser.add_argument(
        "--registration-once",
        action="store_true",
        help="Publish registration once and exit instead of keeping it alive for app reconnect/replay.",
    )
    parser.add_argument("--registration-command", default="")
    parser.add_argument("--registration-timeout", type=float, default=30.0)
    parser.add_argument("--headset-ready-timeout", type=float, default=30.0)
    parser.add_argument("--headset-start-timeout", type=float, default=20.0)
    parser.add_argument("--headset-stop-timeout", type=float, default=20.0)
    parser.add_argument("--clock-sync-interval", type=float, default=30.0)
    parser.add_argument("--clock-sync-timeout", type=float, default=2.0)
    parser.add_argument("--clock-sync-pings", type=int, default=5)
    parser.add_argument("--no-headset", action="store_true")
    parser.add_argument(
        "--export",
        dest="export",
        action="store_true",
        default=True,
        help="Export headset-side metrics from the MR app after measurement ends.",
    )
    parser.add_argument("--no-export", dest="export", action="store_false")
    parser.add_argument("--export-timeout", type=float, default=60.0)
    parser.add_argument("--adb-command", default=default_adb_command())
    parser.add_argument("--quest-package", default="com.UnityTechnologies.com.unity.template.urpblank")
    parser.add_argument("--no-adb-export-fallback", action="store_true")
    parser.add_argument("--no-workload", action="store_true")
    parser.add_argument("--workload-command", default="")
    parser.add_argument("--workload-ready-timeout", type=float, default=15.0)
    parser.add_argument("--workload-host", default="", help="SSH host that runs the synthetic workload near the robot-side transport.")
    parser.add_argument("--remote-sdk-root", default="~/horus_sdk", help="SDK checkout path on --workload-host.")
    parser.add_argument("--remote-results-root", default="/tmp/horus_experiments", help="Run folder root used by remote workload and remote connector roles.")
    parser.add_argument("--no-copy-remote-workload-results", action="store_true")
    parser.add_argument("--no-transport", action="store_true")
    parser.add_argument("--transport-profile", choices=("auto", "none", "local", "direct", "cloud"), default="auto")
    parser.add_argument(
        "--allow-connector-transport",
        action="store_true",
        help="Permit horus_connector/WebRTC transport profiles. Keep disabled for the i-RIM core-platform experiments.",
    )
    parser.add_argument("--transport-role-startup-gap", type=float, default=2.0)
    parser.add_argument("--robot-host", default="arancino")
    parser.add_argument("--cloud-host", default="googlecloud")
    parser.add_argument("--ssh-command", default=default_ssh_command())
    parser.add_argument("--machine-ip", default="auto")
    parser.add_argument("--cloud-ip", default="auto")
    parser.add_argument("--connector-root", default="~/horus_connector")
    parser.add_argument("--local-connector-root", default="/home/omotoye/horus_connector")
    parser.add_argument("--remote-command", action="append", default=[])
    parser.add_argument("--no-failure-injection", action="store_true")
    parser.add_argument("--failure-at", type=float, default=-1.0, help="Override configured failure injection time in seconds.")
    parser.add_argument("--failure-downtime", type=float, default=5.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def failure_settings(workload: Any) -> tuple[str, float]:
    extra = dict(getattr(workload, "extra", {}) or {})
    nested = extra.get("extra")
    if isinstance(nested, dict):
        extra.update(nested)
    failure_type = str(extra.get("failure_type") or "").strip()
    try:
        failure_at = float(extra.get("failure_at_s") or 0.0)
    except (TypeError, ValueError):
        failure_at = 0.0
    return failure_type, failure_at


def registration_command(args: argparse.Namespace, config: Path) -> list[str]:
    if args.registration_command:
        return shlex.split(args.registration_command)
    command = [
        sys.executable,
        str(Path(__file__).resolve().parent / "horus_experiment_registration.py"),
        "--config",
        str(config),
        "--quiet",
        "--timeout",
        str(args.registration_timeout),
    ]
    if args.registration_once:
        command.append("--once")
    return command


def run_registration(args: argparse.Namespace, config: Path, env: dict[str, str]) -> int:
    command = registration_command(args, config)
    print(f"[experiment] registering robots: {' '.join(command)}")
    completed = subprocess.run(command, env=env, cwd=str(SDK_ROOT), text=True, timeout=args.registration_timeout + 10.0)
    return int(completed.returncode)


def start_registration(args: argparse.Namespace, config: Path, env: dict[str, str]) -> subprocess.Popen:
    command = registration_command(args, config)
    print(f"[experiment] registering robots: {' '.join(command)}")
    return start_process(command, env)


def wait_for_operator_ready(args: argparse.Namespace, run_dir: Path) -> bool:
    if args.no_headset or args.ready_mode == "none":
        write_orchestrator_event(run_dir, "ready_wait_skipped", {"reason": "disabled"})
        return True
    if args.ready_mode == "delay":
        delay = max(0.0, args.ready_delay)
        print(f"[experiment] waiting {delay:.1f}s before headset start/registration")
        write_orchestrator_event(run_dir, "ready_wait_started", {"mode": "delay", "delay_s": delay})
        time.sleep(delay)
        write_orchestrator_event(run_dir, "ready_confirmed", {"mode": "delay"})
        return True
    if not sys.stdin.isatty():
        print(
            "[experiment] no interactive terminal is available for the ready prompt; "
            "continuing without waiting. Use --ready-mode delay --ready-delay <seconds> for unattended runs.",
            file=sys.stderr,
        )
        write_orchestrator_event(run_dir, "ready_wait_skipped", {"reason": "non_interactive"})
        return True

    write_orchestrator_event(run_dir, "ready_wait_started", {"mode": "prompt"})
    print()
    print("[experiment] Bridge is running.")
    print("[experiment] Put on the Quest, connect HORUS to this bridge, and create/select the workspace.")
    print("[experiment] Press Enter only after the app shows the workspace and is ready to receive robot registration.")
    try:
        input("[experiment] Ready? Press Enter to continue, or Ctrl+C to cancel. ")
    except KeyboardInterrupt:
        print()
        write_orchestrator_event(run_dir, "ready_wait_cancelled", {"mode": "prompt"})
        return False
    write_orchestrator_event(run_dir, "ready_confirmed", {"mode": "prompt"})
    return True


def wait_for_metric_rows(path: Path, process: subprocess.Popen | None, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() < deadline:
        if path.exists():
            try:
                if metric_row_count(path) > 0:
                    return True
            except Exception:
                pass
        if process is not None and process.poll() is not None:
            return False
        time.sleep(0.1)
    return False


def wait_for_source_metric_stream(
    path: Path,
    stream: str,
    process: subprocess.Popen | None,
    timeout_s: float,
) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() < deadline:
        if source_metric_stream_seen(path, stream):
            return True
        if process is not None and process.poll() is not None:
            return False
        time.sleep(0.1)
    return False


def main() -> int:
    args = parse_args()
    try:
        config = resolve_config(args.experiment)
    except FileNotFoundError as exc:
        print(f"[experiment] {exc}", file=sys.stderr)
        return 2
    workload = load_workload_config(config)
    duration = args.duration if args.duration > 0.0 else workload.duration_s
    warmup = args.warmup if args.warmup >= 0.0 else workload.warmup_s
    if args.duration > 0.0 and abs(args.duration - float(workload.duration_s)) > 1e-6:
        print(
            f"[experiment] WARNING: --duration {args.duration}s overrides config duration_s="
            f"{workload.duration_s}; the committed config is no longer authoritative for this run. "
            "Prefer editing the config so the run reproduces from it (the exact CLI is persisted).",
            file=sys.stderr,
        )
    if args.warmup >= 0.0 and abs(args.warmup - float(workload.warmup_s)) > 1e-6:
        print(
            f"[experiment] WARNING: --warmup {args.warmup}s overrides config warmup_s="
            f"{workload.warmup_s}; the committed config is no longer authoritative for this run.",
            file=sys.stderr,
        )
    profile = effective_transport_profile(args, workload)
    if profile in {"local", "direct", "cloud"} and not args.allow_connector_transport:
        print(
            "[experiment] connector/WebRTC transport is out of scope for the current i-RIM "
            "core-platform experiment set. Continue with E3, or pass "
            "--allow-connector-transport only for a separate RA-L/connector diagnostic run.",
            file=sys.stderr,
        )
        return 2
    if profile in {"direct", "cloud"} and not args.no_workload and not args.workload_host and not args.workload_command:
        print(
            "[experiment] remote transport needs a source on the robot side. "
            f"Use --workload-host {args.robot_host} for synthetic data, "
            "--no-workload if a real source is already publishing there, "
            "or --workload-command for a custom source launcher.",
            file=sys.stderr,
        )
        return 2
    run_dir = run_setup(config, Path(args.results_root), args.run_id, args.notes, duration, warmup)
    _persist_cli_invocation(run_dir)
    _annotate_source_clock(run_dir, args)
    env = experiment_env(run_dir, workload, args.ros_domain_id, args.ros_localhost_only)
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_LOCALHOST_ONLY"] = env["ROS_LOCALHOST_ONLY"]
    os.environ["HORUS_EXPERIMENT_RESULTS_DIR"] = env["HORUS_EXPERIMENT_RESULTS_DIR"]
    os.environ["HORUS_EXPERIMENT_RUN_ID"] = env["HORUS_EXPERIMENT_RUN_ID"]
    required_metrics = {
        "source_metrics.csv": 1 if not args.no_workload else 0,
        "bridge_metrics.csv": 1 if not args.no_bridge else 0,
        "headset_metrics.csv": 1 if not args.no_headset else 0,
    }
    print(f"[experiment] config: {config}")
    print(f"[experiment] run: {run_dir}")
    print(f"[experiment] ROS_DOMAIN_ID={env['ROS_DOMAIN_ID']} ROS_LOCALHOST_ONLY={env['ROS_LOCALHOST_ONLY']}")
    print(f"[experiment] transport_profile={profile}")
    print(f"[experiment] ssh_command={args.ssh_command}")

    if args.dry_run:
        dry_registration = [
            sys.executable,
            str(Path(__file__).resolve().parent / "horus_experiment_registration.py"),
            "--config",
            str(config),
            "--dry-run",
        ]
        subprocess.run(dry_registration, env=env, cwd=str(SDK_ROOT), text=True, check=False)
        print("[experiment] dry run complete")
        return 0

    processes: list[tuple[str, subprocess.Popen]] = []
    rclpy.init()
    control = ExperimentControlNode()
    bridge_process: subprocess.Popen | None = None
    bridge_command: list[str] | None = None
    registration_process: subprocess.Popen | None = None
    transport_processes: list[tuple[str, subprocess.Popen]] = []
    workload_process: subprocess.Popen | None = None
    try:
        if not args.no_bridge:
            bridge_command = shlex.split(args.bridge_command)
            print(f"[experiment] starting bridge: {' '.join(bridge_command)}")
            bridge_process = start_process(bridge_command, env)
            processes.append(("bridge", bridge_process))
            time.sleep(max(0.0, args.bridge_wait))
            if not check_processes_running(run_dir, [("bridge", bridge_process)], phase="bridge_startup"):
                return int(bridge_process.returncode or 1)

        if not wait_for_operator_ready(args, run_dir):
            return 130

        should_register = not args.no_registration and (not args.no_headset or args.force_registration)

        transport_processes = maybe_start_transport_processes(args, workload, run_dir, env)
        processes.extend(transport_processes)
        if transport_processes and args.transport_startup_wait > 0.0:
            time.sleep(args.transport_startup_wait)
            if not check_processes_running(run_dir, transport_processes, phase="transport_startup", allow_successful_exit=True):
                return 1
            transport_processes = live_processes(transport_processes)
            if not run_transport_health_check(args, run_dir, transport_health_checks(args, workload, run_dir)):
                return 1

        workload_duration = (
            max(0.0, warmup)
            + max(0.0, duration)
            + max(1.0, args.workload_ready_timeout)
            + (max(0.0, args.registration_timeout) if should_register else 0.0)
            + (max(0.0, args.headset_ready_timeout) + max(0.0, args.headset_start_timeout) if not args.no_headset else 0.0)
        )
        if not args.no_workload:
            workload_env = env
            if args.workload_host:
                remote_cmd = remote_workload_command(args, config, run_dir, workload, workload_duration, env)
                command = ssh_command(args, args.workload_host, remote_cmd)
            elif args.workload_command:
                command = shlex.split(args.workload_command)
                workload_env = {**env, **clock_anchor_env()}
            else:
                command = [
                    sys.executable,
                    str(Path(__file__).resolve().parent / "horus_synthetic_workload.py"),
                    "--config",
                    str(config),
                    "--duration",
                    str(workload_duration),
                    "--metrics-csv",
                    str(run_dir / "source_metrics.csv"),
                ]
                workload_env = {**env, **clock_anchor_env()}
            print(f"[experiment] starting workload: {' '.join(command)}")
            workload_process = start_process(command, workload_env)
            processes.append(("workload", workload_process))
            if not args.workload_host:
                source_metrics_path = run_dir / "source_metrics.csv"
                if wait_for_metric_rows(source_metrics_path, workload_process, args.workload_ready_timeout):
                    write_orchestrator_event(run_dir, "workload_ready", {"path": str(source_metrics_path)})
                else:
                    write_orchestrator_event(
                        run_dir,
                        "workload_ready_timeout",
                        {"path": str(source_metrics_path), "timeout_s": args.workload_ready_timeout},
                    )
                    return 1
                if should_register:
                    print("[experiment] waiting for source TF before registration")
                    if wait_for_source_metric_stream(source_metrics_path, "tf", workload_process, args.workload_ready_timeout):
                        write_orchestrator_event(run_dir, "workload_tf_ready", {"path": str(source_metrics_path)})
                    else:
                        write_orchestrator_event(
                            run_dir,
                            "workload_tf_ready_timeout",
                            {"path": str(source_metrics_path), "timeout_s": args.workload_ready_timeout},
                        )
                        print("[experiment] source workload did not publish TF before registration", file=sys.stderr)
                        return 1
            elif should_register:
                write_orchestrator_event(
                    run_dir,
                    "workload_tf_ready_unverified",
                    {"reason": "remote_or_custom_workload", "timeout_s": args.workload_ready_timeout},
                )
                time.sleep(max(0.0, min(args.workload_ready_timeout, 3.0)))

        if should_register:
            if args.registration_once:
                rc = run_registration(args, config, env)
                write_orchestrator_event(run_dir, "registration_completed", {"mode": "once", "returncode": rc})
                if rc != 0:
                    print(f"[experiment] registration failed with {rc}", file=sys.stderr)
                    return rc
            else:
                registration_process = start_registration(args, config, env)
                processes.append(("registration", registration_process))
                write_orchestrator_event(
                    run_dir,
                    "registration_started",
                    {"mode": "keep_alive", "pid": registration_process.pid},
                )
                time.sleep(min(2.0, max(0.0, args.registration_timeout)))
                if registration_process.poll() is not None:
                    rc = int(registration_process.returncode or 0)
                    write_orchestrator_event(run_dir, "registration_exited", {"mode": "keep_alive", "returncode": rc})
                    if rc != 0:
                        print(f"[experiment] registration failed with {rc}", file=sys.stderr)
                        return rc
        elif args.no_registration:
            write_orchestrator_event(run_dir, "registration_skipped", {"reason": "disabled_by_argument"})
        else:
            write_orchestrator_event(run_dir, "registration_skipped", {"reason": "no_headset"})

        if registration_process is not None and registration_process.poll() not in (None, 0):
            rc = int(registration_process.returncode or 1)
            write_orchestrator_event(run_dir, "registration_failed_before_measurement", {"returncode": rc})
            print(f"[experiment] registration exited before measurement with {rc}", file=sys.stderr)
            return rc

        if not args.no_headset:
            if not send_action_until_status(
                control,
                run_dir,
                "status",
                {"idle", "recording", "started", "stopped"},
                timeout_s=args.headset_ready_timeout,
                include_existing_status=False,
            ):
                print("[experiment] headset recorder did not answer the status request", file=sys.stderr)
                return 3
            if collect_clock_sync_sample(control, run_dir, "before_start") is None:
                print("[experiment] headset recorder did not answer the clock sync request", file=sys.stderr)
                return 3
            if not send_action_until_status(
                control,
                run_dir,
                "start",
                {"started", "recording"},
                timeout_s=args.headset_start_timeout,
                run_id=run_dir.name,
                experiment=workload.experiment,
                condition=workload.condition,
            ):
                print("[experiment] headset recorder did not acknowledge start", file=sys.stderr)
                return 3
        else:
            control.send_action(
                "start",
                repeats=3,
                run_id=run_dir.name,
                experiment=workload.experiment,
                condition=workload.condition,
            )

        if not args.no_workload:
            if not args.workload_host:
                write_workload_start_signal(run_dir, workload)
            control.send_action(
                "workload_start",
                repeats=20,
                run_id=run_dir.name,
                experiment=workload.experiment,
                condition=workload.condition,
            )
            expected_streams = expected_source_streams_after_start(workload)
            if expected_streams and workload_process is not None and not args.workload_host and not args.workload_command:
                source_metrics_path = run_dir / "source_metrics.csv"
                for stream in expected_streams:
                    if wait_for_source_metric_stream(
                        source_metrics_path,
                        stream,
                        workload_process,
                        args.workload_ready_timeout,
                    ):
                        write_orchestrator_event(
                            run_dir,
                            "workload_stream_ready",
                            {"stream": stream, "path": str(source_metrics_path)},
                        )
                    else:
                        write_orchestrator_event(
                            run_dir,
                            "workload_stream_ready_timeout",
                            {
                                "stream": stream,
                                "path": str(source_metrics_path),
                                "timeout_s": args.workload_ready_timeout,
                            },
                        )
                        print(
                            f"[experiment] source workload did not publish expected stream before measurement: {stream}",
                            file=sys.stderr,
                        )
                        return 1

        failure_type, failure_at = failure_settings(workload)
        if args.failure_at >= 0.0:
            failure_at = args.failure_at
        failure_injected = False
        if warmup > 0.0:
            time.sleep(max(0.0, warmup))
        if not args.no_headset:
            collect_clock_sync_sample(
                control,
                run_dir,
                "measurement_start",
                timeout_s=max(0.25, args.clock_sync_timeout),
                ping_count=max(1, int(args.clock_sync_pings)),
            )
        write_orchestrator_event(run_dir, "measurement_start", {"warmup_s": warmup, "duration_s": duration})
        control.send_action(
            "measurement_start",
            repeats=10,
            run_id=run_dir.name,
            experiment=workload.experiment,
            condition=workload.condition,
        )
        if not args.no_headset:
            control.send_action("mark", repeats=1, name="measurement_start", notes=f"warmup_s={warmup}")
        measurement_deadline = time.monotonic() + max(0.0, duration)
        next_clock_sync_at = (
            time.monotonic() + max(0.0, args.clock_sync_interval)
            if not args.no_headset and args.clock_sync_interval > 0.0
            else float("inf")
        )
        while time.monotonic() < measurement_deadline:
            if workload_process is not None and workload_process.poll() is not None:
                break
            if registration_process is not None and registration_process.poll() not in (None, 0):
                rc = int(registration_process.returncode or 1)
                write_orchestrator_event(run_dir, "registration_failed_during_measurement", {"returncode": rc})
                print(f"[experiment] registration exited during measurement with {rc}", file=sys.stderr)
                return rc
            if transport_processes:
                if not check_processes_running(run_dir, transport_processes, phase="measurement", allow_successful_exit=True):
                    return 1
                transport_processes = live_processes(transport_processes)
            elapsed = duration - max(0.0, measurement_deadline - time.monotonic())
            if (
                not args.no_failure_injection
                and not failure_injected
                and failure_type == "bridge_restart"
                and failure_at > 0.0
                and elapsed >= failure_at
                and bridge_process is not None
                and bridge_command is not None
            ):
                failure_injected = True
                write_orchestrator_event(run_dir, "failure_injection_started", {"type": failure_type, "elapsed_s": elapsed})
                if not args.no_headset:
                    control.send_action("mark", name="failure_injection_started", notes=f"type={failure_type}")
                stop_process(bridge_process, "bridge")
                time.sleep(max(0.0, args.failure_downtime))
                bridge_process = start_process(bridge_command, env)
                processes.append(("bridge_restarted", bridge_process))
                write_orchestrator_event(
                    run_dir,
                    "failure_injection_recovered",
                    {"type": failure_type, "elapsed_s": elapsed + args.failure_downtime, "downtime_s": args.failure_downtime},
                )
                if not args.no_headset:
                    control.send_action("mark", name="failure_injection_recovered", notes=f"downtime_s={args.failure_downtime}")
            if not args.no_headset and time.monotonic() >= next_clock_sync_at:
                sample_index = int(max(0.0, duration - max(0.0, measurement_deadline - time.monotonic())) // max(1.0, args.clock_sync_interval))
                collect_clock_sync_sample(
                    control,
                    run_dir,
                    f"measurement_{sample_index}",
                    timeout_s=max(0.25, args.clock_sync_timeout),
                    ping_count=max(1, int(args.clock_sync_pings)),
                )
                next_clock_sync_at = time.monotonic() + max(1.0, args.clock_sync_interval)
            rclpy.spin_once(control, timeout_sec=0.1)
        write_orchestrator_event(run_dir, "measurement_end", {"duration_s": duration})
        if not args.no_headset:
            control.send_action("mark", repeats=1, name="measurement_end", notes=f"duration_s={duration}")

        workload_stop_requested = False
        workload_failed_after_measurement = False
        if workload_process is not None and workload_process.poll() is None:
            write_orchestrator_event(run_dir, "workload_stop_requested", {"reason": "measurement_complete"})
            workload_stop_requested = True
            stop_process(workload_process, "workload", timeout=10.0)
        if workload_process is not None and args.workload_host:
            copy_remote_workload_results(args, run_dir)
        if workload_process is not None and workload_process.returncode not in (0, None):
            if workload_stop_requested:
                workload_failed_after_measurement = True
                write_orchestrator_event(
                    run_dir,
                    "workload_exit_ignored_after_measurement",
                    {"returncode": int(workload_process.returncode)},
                )
                print(
                    f"[experiment] workload exited with {workload_process.returncode} during shutdown after measurement",
                    file=sys.stderr,
                )
            else:
                write_orchestrator_event(
                    run_dir,
                    "workload_failed_after_measurement",
                    {"returncode": int(workload_process.returncode)},
                )

        if not args.no_headset:
            if collect_clock_sync_sample(control, run_dir, "after_measurement") is None:
                write_orchestrator_event(
                    run_dir,
                    "after_measurement_clock_sync_missed",
                    {"reason": "continuing_to_stop_and_export_headset"},
                )
                print(
                    "[experiment] headset recorder did not answer the final clock sync request; continuing to stop/export",
                    file=sys.stderr,
                )
            if not send_action_until_status(
                control,
                run_dir,
                "stop",
                {"stopped", "idle"},
                timeout_s=args.headset_stop_timeout,
                run_id=run_dir.name,
            ):
                print("[experiment] headset recorder did not acknowledge stop", file=sys.stderr)
                return 3

        if args.export and not args.no_headset:
            collect_headset_export(
                control,
                run_dir,
                args.export_timeout,
                adb_command="" if args.no_adb_export_fallback else args.adb_command,
                quest_package=args.quest_package,
            )
        elif args.no_headset:
            write_orchestrator_event(run_dir, "headset_export_skipped", {"reason": "no_headset"})
        else:
            write_orchestrator_event(run_dir, "headset_export_skipped", {"reason": "disabled_by_argument"})
        write_summary(run_dir)
        quality = validate_run_directory(run_dir, required_metrics)
        if not quality.get("ok", False):
            for error in quality.get("errors", []):
                print(f"[experiment] data quality error: {error}", file=sys.stderr)
            return 3
        if workload_process is not None and workload_process.returncode not in (0, None) and not workload_failed_after_measurement:
            print(f"[experiment] workload exited with {workload_process.returncode}", file=sys.stderr)
            return int(workload_process.returncode)
    finally:
        stop_transport_services(args, workload, run_dir, env)
        for name, process in reversed(processes):
            stop_process(process, name)
        collect_webrtc_metrics(run_dir, workload)
        write_summary(run_dir)
        validate_run_directory(run_dir, required_metrics)
        control.destroy_node()
        rclpy.shutdown()

    print(f"[experiment] done: {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
