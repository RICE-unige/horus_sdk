"""Manage only the local HORUS hospital demo and record every child process."""

import argparse
from datetime import datetime, timezone
import importlib
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from configuration import write_configuration, prepare_description

SOURCE = Path(__file__).resolve().parent
SDK = SOURCE.parents[1]
DEFAULT_STORAGE = Path("/media/omotoye/New Volume/HORUS/data/hospital")
STORAGE = Path(os.environ.get("HORUS_HOSPITAL_DATA_ROOT", str(DEFAULT_STORAGE))).expanduser()
STATE = STORAGE / "current.json"
BOOT_ID = Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def positive_resolution(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("Voxel size must be a finite positive number of metres")
    return number


def process_identity(pid):
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        return stat[stat.rfind(")")+2:].split()[19]
    except (FileNotFoundError, PermissionError):
        return None


def live_state():
    if not STATE.is_file():
        return None
    value = json.loads(STATE.read_text())
    identity = process_identity(value["pid"])
    return value if (value.get("boot_id") == BOOT_ID and identity is not None
                     and identity == value.get("start_ticks")) else None


def check():
    from ament_index_python.packages import get_package_prefix
    for module in ("rclpy", "cv2", "numpy", "PIL", "horus", "tf2_ros", "nvblox_msgs.msg"):
        importlib.import_module(module)
    for package in ("horus_backend", "horus_unity_bridge", "nvblox_ros", "nav2_controller",
                    "nav2_planner", "nav2_bt_navigator", "robot_state_publisher"):
        get_package_prefix(package)
    capabilities_file = Path(get_package_prefix("horus_unity_bridge")) / "share/horus_unity_bridge/build_capabilities.json"
    capabilities = json.loads(capabilities_file.read_text()) if capabilities_file.is_file() else {}
    if capabilities.get("webrtc") is not True:
        raise RuntimeError("The installed HORUS bridge needs a verified WebRTC-enabled build for this demo.")
    slam = Path(get_package_prefix("slam_toolbox"))
    if not (slam / "lib/slam_toolbox/decentralized_multirobot_slam_toolbox_node").is_file():
        raise RuntimeError("The pinned cooperative SLAM overlay is not built/sourced.")
    isaac = Path(os.environ.get("HORUS_ISAAC_HOME", str(Path.home()/"isaac")))
    assets = Path(os.environ.get("HORUS_ISAAC_ASSET_ROOT",
        "/media/omotoye/New Volume/IsaacSim/assets-6.2.0/Assets/Isaac/6.2"))
    if not (isaac / "env_isaacsim/bin/isaacsim").is_file():
        raise RuntimeError(f"Shared Isaac Sim is unavailable: {isaac}")
    if not (assets / "Isaac/Samples/ROS2/Scenario/multiple_robot_carter_hospital_navigation.usd").is_file():
        raise RuntimeError(f"Hospital asset unavailable: {assets}")
    model_root = Path(os.environ.get("HORUS_HOSPITAL_MODEL_ROOT",
        "~/.cache/horus/robot_description_sources/nova_carter")).expanduser()
    model = model_root / "nova_carter_description/urdf/horus_nova_carter_static_shell.urdf"
    if not model.is_file():
        raise RuntimeError(f"Carter model asset unavailable: {model}")
    storage_parent = STORAGE if STORAGE.exists() else STORAGE.parent
    if not storage_parent.is_dir():
        raise RuntimeError(f"Data drive is not mounted: {storage_parent}")
    return {"isaac": str(isaac), "assets": str(assets), "slam": str(slam),
            "model": str(model), "model_root": str(model_root),
            "domain": int(os.environ["ROS_DOMAIN_ID"]), "storage": str(STORAGE),
            "bridge_capabilities": capabilities}


def owned_stop(pid, start_ticks, sig, marker=None):
    identity = process_identity(pid)
    owned = identity is not None and start_ticks is not None and identity == start_ticks
    if not owned and marker:
        expected = ("HORUS_HOSPITAL_RUN=" + str(marker)).encode()
        for entry in Path("/proc").iterdir():
            if not entry.name.isdecimal():
                continue
            try:
                if os.getpgid(int(entry.name)) == pid and expected in (entry / "environ").read_bytes().split(b"\0"):
                    owned = True
                    break
            except (ProcessLookupError, FileNotFoundError, PermissionError):
                continue
    if owned:
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            pass


class Processes:
    def __init__(self, directory, environment):
        self.directory = directory
        self.environment = environment
        self.children = []

    def start(self, name, command, finite=False, append=False):
        with (self.directory / f"{name}.log").open("ab" if append else "wb") as output:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                cwd=SDK, env=self.environment, start_new_session=True)
        row = {"name": name, "pid": process.pid, "start_ticks": process_identity(process.pid),
               "command": command, "finite": finite}
        self.children.append((process, row))
        (self.directory / "processes.json").write_text(json.dumps([entry for _, entry in self.children], indent=2))
        return process

    def restart(self, name):
        index = next(i for i, (_, row) in enumerate(self.children) if row["name"] == name)
        process, row = self.children[index]
        owned_stop(process.pid, row["start_ticks"], signal.SIGINT, self.directory)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            owned_stop(process.pid, row["start_ticks"], signal.SIGKILL, self.directory)
            process.wait(timeout=5)
        self.children.pop(index)
        return self.start(name, row["command"], finite=row["finite"], append=True)

    def verify(self):
        for process, row in self.children:
            if process.poll() is not None and not (row["finite"] and process.returncode == 0):
                raise RuntimeError(f"{row['name']} exited({process.returncode}); see {self.directory / (row['name']+'.log')}")

    def stop(self):
        for process, row in reversed(self.children):
            owned_stop(process.pid, row["start_ticks"], signal.SIGINT, self.directory)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and any(p.poll() is None for p, _ in self.children):
            time.sleep(.2)
        for process, row in reversed(self.children):
            owned_stop(process.pid, row["start_ticks"], signal.SIGTERM, self.directory)
        for process, row in reversed(self.children):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                owned_stop(process.pid, row["start_ticks"], signal.SIGKILL, self.directory)
                process.wait(timeout=5)


def wait_for(predicate, processes, timeout, description, cancelled=lambda: False):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        processes.verify()
        if cancelled():
            raise InterruptedError("Hospital startup cancelled")
        if predicate():
            return
        time.sleep(.5)
    raise RuntimeError(f"Timed out waiting for {description}; inspect {processes.directory}")


def serve(directory, voxel_size=.05):
    lock = (STORAGE / "demo.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    inventory = check()
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "inventory.json").write_text(json.dumps(inventory, indent=2))
    write_configuration(directory / "config", voxel_size)
    description = prepare_description(inventory["model"], directory / "carter.urdf")
    scene = directory / "hospital.usda"
    subprocess.run([str(Path(inventory["isaac"]) / "env_isaacsim/bin/python"),
        str(SOURCE / "prepare_scene.py"), "--asset-root", inventory["assets"], "--output", str(scene)], check=True)
    environment = dict(os.environ, HORUS_HOSPITAL_SOURCE=str(SOURCE), HORUS_HOSPITAL_RUN=str(directory),
                       HORUS_HOSPITAL_SCENE=str(scene), HORUS_HOSPITAL_URDF=str(description))
    processes = Processes(directory, environment)
    stop_requested = False
    refresh_requested = False

    def stop_signal(_sig, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    def refresh_signal(_sig, _frame):
        nonlocal refresh_requested
        refresh_requested = True
    signal.signal(signal.SIGUSR1, refresh_signal)
    state = {"pid": os.getpid(), "start_ticks": process_identity(os.getpid()), "boot_id": BOOT_ID,
             "run": str(directory), "phase": "starting", "voxel_size_m": voxel_size,
             "registration_refresh": True}
    STATE.write_text(json.dumps(state, indent=2))
    try:
        for port in (int(os.environ.get("HORUS_HOSPITAL_KIT_PORT", "8226")), 10000, 10001):
            with socket.socket() as probe:
                if probe.connect_ex(("127.0.0.1", port)) == 0:
                    raise RuntimeError(f"Port {port} belongs to another service; it was left untouched. Stop that demo first.")
        processes.start("simulation", ["bash", str(SOURCE / "run_sim.sh")])
        wait_for(lambda: (directory / "simulation.json").is_file(), processes, 300,
                 "simulation sensor startup", lambda: stop_requested)
        processes.start("display", [sys.executable, str(SOURCE / "ros_support.py")])
        processes.start("robotics", ["ros2", "launch", str(SOURCE / "robotics.launch.py")])
        processes.start("tasks", [sys.executable, str(SOURCE / "navigation_support.py")])
        processes.start("mesh", [sys.executable, str(SDK / "python/examples/tools/nvblox_mesh_to_horus.py"),
            "--input-topic", "/hospital_mapper/mesh", "--output-topic", "/horus/hospital/mesh", "--blocks-per-axis", "8"])
        with socket.socket() as probe:
            if probe.connect_ex(("127.0.0.1", 10000)) == 0:
                raise RuntimeError("Another HORUS bridge owns port10000; left untouched. Stop that demo first.")
        processes.start("bridge", ["ros2", "launch", "horus_backend", "horus_complete_backend.launch.py"])
        def bridge_ready():
            with socket.socket() as probe:
                return probe.connect_ex(("127.0.0.1", 10000)) == 0
        wait_for(bridge_ready, processes, 60, "HORUS bridge", lambda: stop_requested)
        processes.start("registration", [sys.executable, str(SDK / "python/examples/hospital_fleet_registration.py")])
        state["phase"] = "running_checks_pending"
        STATE.write_text(json.dumps(state, indent=2))
        probe_file = directory / "startup-streams.json"
        probe = processes.start("startup_check", [sys.executable, str(SOURCE / "check_streams.py"),
            "--duration", "20", "--output", str(probe_file)], finite=True)
        wait_for(lambda: probe.poll() is not None, processes, 40, "local stream verification", lambda: stop_requested)
        if probe.returncode or not probe_file.is_file():
            raise RuntimeError("Hospital stream verification failed; see startup_check.log")
        receipt = json.loads(probe_file.read_text())
        if receipt["missing_topics"]:
            raise RuntimeError(f"Missing local streams: {receipt['missing_topics']}")
        if receipt.get("invalid_topics"):
            raise RuntimeError(f"Invalid local streams: {receipt['invalid_topics']}")
        state["phase"] = "local_streams_ready"
        STATE.write_text(json.dumps(state, indent=2))
        print(f"Hospital local streams ready; Quest connection remains independent. Logs: {directory}", flush=True)
        while not stop_requested:
            if refresh_requested:
                refresh_requested = False
                processes.restart("registration")
                print("Refreshed SDK registration only; simulation and map retained.", flush=True)
            processes.verify()
            if time.time() - (directory / "simulation.json").stat().st_mtime > 25:
                raise RuntimeError("Simulation status stopped updating")
            time.sleep(1)
    except InterruptedError:
        state["phase"] = "stopped"
    except Exception as error:
        state.update(phase="failed", error=str(error))
        print(str(error), file=sys.stderr, flush=True)
        (directory / "failure.json").write_text(json.dumps(state, indent=2))
        raise
    finally:
        processes.stop()
        state["phase"] = "stopped" if state["phase"] != "failed" else "failed"
        (directory / "exit.json").write_text(json.dumps(state, indent=2))
        STATE.write_text(json.dumps(state, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", choices=("start", "stop", "status", "check", "verify", "refresh-registration", "render-profile", "serve"), default="start")
    parser.add_argument("profile", nargs="?", choices=("full", "lod", "spacewarp", "combined"))
    parser.add_argument("--run", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--voxel-size", type=positive_resolution, default=.05,
                        help="3D-map voxel spacing in metres for a fresh start (default: 0.05)")
    args = parser.parse_args()
    if args.command == "check":
        print(json.dumps(check(), indent=2))
        return
    if args.command == "status":
        value = live_state()
        print(json.dumps(value or {"phase": "stopped"}, indent=2))
        return
    if args.command in ("refresh-registration", "render-profile"):
        value = live_state()
        if not value:
            raise SystemExit("No managed demo is running. Start it with ./hospital-demo start.")
        if value.get("phase") != "local_streams_ready":
            raise SystemExit("Wait for local_streams_ready before refreshing registration.")
        if not value.get("registration_refresh"):
            raise SystemExit("This running supervisor predates registration refresh. Save the map and restart the demo first.")
        if args.command == "render-profile":
            if args.profile is None:
                parser.error("render-profile requires full, lod, spacewarp or combined")
            profile = {"lod_enabled": True, "force_full_detail": args.profile in ("full", "spacewarp"),
                       "lod_error_pixels": 1.0, "lod_memory_mib": 96,
                       "spacewarp_enabled": args.profile in ("spacewarp", "combined")}
            destination = Path(value["run"]) / "map-render-profile.json"
            temporary = destination.with_suffix(".tmp")
            temporary.write_text(json.dumps(profile, indent=2) + "\n")
            temporary.replace(destination)
            print(f"Selected {args.profile}; LOD changes displayed geometry, SpaceWarp synthesizes frames.")
        owned_stop(value["pid"], value["start_ticks"], signal.SIGUSR1)
        print("Requested SDK registration refresh; simulation and mapping remain running.")
        return
    if args.command == "verify":
        value = live_state()
        if not value:
            raise SystemExit("No managed demo is running. Start it with ./hospital-demo start.")
        output = Path(value["run"]) / ("streams_" + datetime.now(timezone.utc).strftime("%H%M%S") + ".json")
        subprocess.run([sys.executable, str(SOURCE / "check_streams.py"), "--duration", "20", "--output", str(output)], check=True)
        receipt = json.loads(output.read_text())
        print(f"Actual ROS receipt data: {output}")
        if receipt["missing_topics"]:
            raise SystemExit(f"Missing streams: {receipt['missing_topics']}")
        if receipt.get("invalid_topics"):
            raise SystemExit(f"Invalid streams: {receipt['invalid_topics']}")
        return
    if args.command == "stop":
        value = live_state()
        if value:
            owned_stop(value["pid"], value["start_ticks"], signal.SIGINT)
            print("Stopping owned hospital demo; saved run/logs are retained.")
            deadline = time.monotonic()+40
            while live_state() and time.monotonic() < deadline:
                time.sleep(.2)
            if live_state():
                raise SystemExit("Shutdown is still pending; inspect the run's supervisor.log before restarting.")
        else:
            print("No managed hospital demo is running.")
        return
    if args.command == "serve":
        if not args.run:
            parser.error("serve needs --run")
        serve(args.run, args.voxel_size)
        return
    if live_state():
        print("Hospital demo is already running. Use ./hospital-demo status.")
        return
    check()
    STORAGE.mkdir(parents=True, exist_ok=True)
    directory = STORAGE / datetime.now(timezone.utc).strftime("hospital_%Y%m%dT%H%M%S_%fZ")
    directory.mkdir()
    with (directory / "supervisor.log").open("wb") as output:
        subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "serve", "--run", str(directory),
                          "--voxel-size", str(args.voxel_size)],
            stdout=output, stderr=subprocess.STDOUT, cwd=SDK, env=os.environ.copy(), start_new_session=True)
    print(f"Starting hospital demo. Logs: {directory}")
    print("Use ./hospital-demo status. Remote connector testing is not enabled.")


if __name__ == "__main__":
    main()
