#!/usr/bin/env python3
"""Launch real HoloLens reception, measured mapping and HORUS registration."""
import argparse
from datetime import datetime, timezone
import importlib
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from examples.tools.field_teammate_scene import scene_frames
from examples.tools.field_map_source import resolve_map_source


EXAMPLES = Path(__file__).resolve().parents[1]
CONNECTOR = Path(os.environ.get("HORUS_CONNECTOR_ROOT", "~/horus_connector")).expanduser()
PROFILE_VOXELS = {"quality": (0.01, 0.02), "balanced": (0.02, 0.02),
                  "3cm": (0.03, 0.03), "performance": (0.05, 0.05)}
MESH_RENDER_PROFILES = ("off", "lod", "spacewarp", "combined")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", nargs="?", help="HoloLens IPv4 address from its HORUS Field status panel")
    parser.add_argument("--check", action="store_true", help="Check local dependencies; also check headset ports if an IP is supplied")
    parser.add_argument("--receive-only", action="store_true", help="Pose/video/registration only, without Research Mode depth or mapping")
    parser.add_argument("--estimated-depth", action="store_true", help="Also map RGB-guided physical-depth completion as a separate provisional layer")
    parser.add_argument("--adaptive", action="store_true", help="Keep a 1 cm source; show adaptive 1 cm interests / 3 cm context on Quest")
    parser.add_argument("--map-source", choices=("measured", "estimated"),
                        help="Map shown on Quest and initially in Rerun; defaults to estimated with --estimated-depth, otherwise measured")
    parser.add_argument("--mapping-memory", choices=("device", "unified"), default="device",
                        help="Map layer allocation; unified requires the optional HORUS mapper overlay and can use host RAM")
    parser.add_argument("--mapping-profile", choices=tuple(PROFILE_VOXELS), default="quality",
                        help="quality: 1 cm measured / 2 cm estimated; balanced: 2 cm for both; 3cm: 3 cm for both; performance: 5 cm for both; all retain full map coverage")
    parser.add_argument("--mesh-render-profile", choices=MESH_RENDER_PROFILES, default="combined",
                        help="Quest mesh presentation; combined enables distant-display LOD and SpaceWarp without changing source geometry")
    parser.add_argument("--depth-backend", choices=("lingbot", "promptda_small", "promptda_large"), default="lingbot")
    parser.add_argument("--estimated-range", type=float, default=10.0,
                        help="Maximum provisional estimated depth in metres, not a physical sensor range (default: 10)")
    parser.add_argument("--depth-runtime", type=Path,
                        default=Path(os.environ.get("HORUS_FIELD_DEPTH_RUNTIME", "~/.venvs/horus-field-mapping-depth")).expanduser())
    parser.add_argument("--no-view", action="store_true", help="Do not open a PC map viewer")
    parser.add_argument("--visualizer", choices=("rviz", "rerun", "none"), default="rviz",
                        help="PC map viewer (default: rviz); ignored for receive-only")
    parser.add_argument("--rerun-runtime", type=Path,
                        default=Path(os.environ.get("HORUS_RERUN_RUNTIME", "~/.local/share/horus/rerun-0.34.1")).expanduser(),
                        help="Separate Rerun Python environment, used only with --visualizer rerun")
    parser.add_argument("--name", default="field_teammate_1")
    parser.add_argument("--duration", type=float, help="Optional bounded run, in seconds after launch")
    return parser


def preflight(mapping=True, visualizer="none", rerun_runtime=None, estimated=False, depth_runtime=None, backend="lingbot", mapping_memory="device"):
    from ament_index_python.packages import get_package_prefix
    modules = ["rclpy", "cv2", "numpy", "horus", "sensor_msgs.msg", "nav_msgs.msg", "tf2_ros"]
    if mapping:
        modules += ["av", "hl2ss", "hl2ss_lnm", "hl2ss_3dcv", "horus_field_mapping.long_throw_bridge", "nvblox_msgs.msg"]
    for module in modules:
        importlib.import_module(module)
    relay = CONNECTOR / "scripts/field_teammate_hololens_relay.py"
    if not relay.is_file():
        raise RuntimeError(f"Canonical connector relay not found: {relay}")
    result = {"connector": str(relay), "python": sys.executable,
              "domain": os.environ.get("ROS_DOMAIN_ID"), "mapping": mapping}
    if mapping:
        package_name, executable_name = mapper_program(mapping_memory)
        try:
            package = Path(get_package_prefix(package_name))
        except LookupError as error:
            raise RuntimeError(f"Mapper package {package_name} is unavailable. "
                               "Build and source the optional HORUS memory overlay for unified allocation.") from error
        executable = package / "lib" / package_name / executable_name
        if not executable.is_file():
            raise RuntimeError(f"nvblox executable missing: {executable}")
        libraries = subprocess.run(["ldd", str(executable)], text=True, capture_output=True, timeout=15, check=True)
        missing = [line.strip() for line in libraries.stdout.splitlines() if "not found" in line]
        if missing:
            raise RuntimeError(f"nvblox libraries unavailable: {missing}")
        result["nvblox"] = str(executable)
        result["mapping_memory_type"] = mapping_memory
        result["mapping_package"] = get_package_prefix("horus_field_mapping")
        if estimated:
            runtime = Path(depth_runtime).expanduser().resolve()
            executable = runtime / "bin/python"
            if not executable.is_file():
                raise RuntimeError(f"Optional RGB-D completion runtime is missing: {executable}")
            check = subprocess.run([str(executable), "-c",
                "import sys,torch,cv2; from huggingface_hub import hf_hub_download; "
                "from horus_field_mapping.depth_estimators import ESTIMATORS; "
                "spec=ESTIMATORS[sys.argv[1]]; assert torch.cuda.is_available(), 'CUDA unavailable'; "
                "print(hf_hub_download(spec.model_id, filename=spec.filename, revision=spec.revision, local_files_only=True))",
                backend], text=True, capture_output=True, timeout=30)
            if check.returncode:
                raise RuntimeError(f"RGB-D model/runtime preflight failed: {check.stderr.strip()}")
            result["depth_runtime"] = str(runtime)
            result["depth_backend"] = backend
            result["depth_checkpoint"] = check.stdout.strip()
        if visualizer == "rerun":
            runtime = Path(rerun_runtime).expanduser().resolve()
            for executable in (runtime / "bin/python", runtime / "bin/rerun"):
                if not executable.is_file() or not os.access(executable, os.X_OK):
                    raise RuntimeError(f"Rerun runtime is unavailable: {executable}")
            check = subprocess.run([str(runtime / "bin/python"), "-c",
                "import rerun, rerun.blueprint, rclpy; from nvblox_msgs.msg import Mesh; print(rerun.__version__)"],
                text=True, capture_output=True, timeout=20)
            if check.returncode:
                raise RuntimeError(f"Rerun ROS runtime check failed: {check.stderr.strip()}")
            result["rerun_runtime"] = str(runtime)
            result["rerun_version"] = check.stdout.strip()
    return result


def mapper_program(memory):
    if memory == "device":
        return "nvblox_ros", "nvblox_node"
    if memory == "unified":
        return "horus_nvblox_memory", "horus_nvblox_node"
    raise ValueError("Mapping memory must be device or unified")


def probe_headset(host, mapping=True, verify_protocol=True):
    """Verify HORUS stream identity, not merely arbitrary listeners on the ports."""
    from field_teammate_hololens_relay import DeadlineReader
    result = {}
    for port, greeting in ((3810, b"HORUS_PV_STREAM_V1\n"), (3814, b"HORUS_POSE_STREAM_V1\n"),
                           (3816, b"HORUS_UMQ_STREAM_V1\n")):
        try:
            with socket.create_connection((host, port), timeout=3) as peer:
                if verify_protocol and DeadlineReader(peer, timeout=3).readline() != greeting:
                    raise RuntimeError(f"{host}:{port} is not the expected HORUS Field endpoint")
        except OSError as error:
            raise RuntimeError(f"HORUS Field endpoint {host}:{port} did not respond: {error}. "
                "Keep the app open and the HoloLens awake on the same LAN.") from error
        result[str(port)] = "HORUS protocol verified" if verify_protocol else "TCP reachable; awaiting stream validation"
    if mapping:
        for port in (3805, 3809):
            try:
                with socket.create_connection((host, port), timeout=3):
                    pass
            except OSError as error:
                raise RuntimeError(f"Research Mode endpoint {host}:{port} unavailable: {error}. "
                    "Enable Research Mode and reopen HORUS Field, or use --receive-only for pose/video.") from error
            result[str(port)] = "Research Mode endpoint reachable; calibration/tracking checked during streaming"
    return result


def reject_existing_session(mapping=True, proc_root=Path("/proc"), name="field_teammate_1"):
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    for path in proc_root.glob("[0-9]*/cmdline"):
        try:
            arguments = [part.decode() for part in path.read_bytes().split(b"\0") if part]
            if not any(Path(arg).name == "field_teammate_hololens_relay.py" for arg in arguments):
                continue
            environment = dict(part.split(b"=", 1) for part in (path.parent / "environ").read_bytes().split(b"\0") if b"=" in part)
            names = argparse.ArgumentParser(add_help=False)
            names.add_argument("--name", default="field_teammate_1")
            existing, _ = names.parse_known_args(arguments)
            if environment.get(b"ROS_DOMAIN_ID", b"0").decode() == domain and existing.name == name:
                raise RuntimeError(f"A Field relay already runs in domain{domain} (PID{path.parent.name}); stop its own launch first")
        except (OSError, UnicodeError):
            continue
    if mapping:
        import fcntl
        lock_path = Path(f"/tmp/horus_dense_mapping_domain_{domain}.lock")
        if lock_path.is_file():
            with lock_path.open("rb") as lock:
                try:
                    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                    fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
                except BlockingIOError as error:
                    raise RuntimeError(f"An active mapping session already owns domain{domain}; stop its own launch first") from error


class Children:
    def __init__(self, directory):
        self.directory = directory
        self.processes = []

    def start(self, name, command, *, console=False):
        log_path = self.directory / f"{name}.log"
        with log_path.open("wb") as log:
            process = subprocess.Popen(command, stdout=None if console else log,
                stderr=None if console else subprocess.STDOUT, env=os.environ.copy(), start_new_session=True)
        self.processes.append((name, process))
        return process

    def stop(self):
        # Only process groups created by this launch are signalled; no pkill/port cleanup.
        for _, process in reversed(self.processes):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for name, process in reversed(self.processes):
            try:
                process.wait(timeout=90 if name == "adaptive" else 8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
                    except ProcessLookupError:
                        pass
                except ProcessLookupError:
                    pass
        # A launcher can exit before its grandchildren. Its owned group must stop too.
        remaining = []
        for _, process in self.processes:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                remaining.append(process.pid)
            except ProcessLookupError:
                pass
        if remaining:
            time.sleep(.5)
            for pid in remaining:
                try:
                    os.killpg(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def command_plan(host, directory, name="field_teammate_1", mapping=True, view=True,
                 visualizer=None, rerun_runtime=None, estimated=False, depth_runtime=None,
                 backend="lingbot", estimated_range=10.0, mapping_profile="quality", adaptive=False,
                 mapping_memory="device", map_source=None, mesh_render_profile="combined"):
    source = resolve_map_source(estimated, map_source)
    mapper_program(mapping_memory)
    if mapping_profile not in PROFILE_VOXELS:
        raise ValueError("Mapping profile must be quality, balanced, 3cm or performance")
    if mesh_render_profile not in MESH_RENDER_PROFILES:
        raise ValueError("Unknown mesh render profile")
    measured_voxel, estimated_voxel = PROFILE_VOXELS[mapping_profile]
    if adaptive:
        if not mapping or mapping_profile != "quality":
            raise ValueError("adaptive viewing needs mapping with the quality source profile")
        measured_voxel = estimated_voxel = .01
    python = sys.executable
    relay = [python, str(CONNECTOR / "scripts/field_teammate_hololens_relay.py"),
        "--hololens-host", host, "--name", name, "--video-profile", "mapping", "--disable-voice"]
    registration = [python, str(EXAMPLES / "field_teammate_mapping_registration.py"), "--name", name]
    if mapping:
        registration += ["--map-source", source, "--mesh-render-profile", mesh_render_profile]
    if adaptive:
        registration.append("--adaptive")
    health = [python, str(EXAMPLES / "tools/field_teammate_health.py"), "--name", name,
              "--output", str(directory / "reception.ndjson")]
    mesh_health = [python, str(EXAMPLES / "tools/field_teammate_health.py"), "--name", name,
                   "--output", str(directory / "mesh_reception.ndjson"), "--mesh-worker",
                   "--mesh-status-topic", f"/{name}/research/mesh_reception/status"]
    if mapping:
        health += ["--mesh-status-topic", f"/{name}/research/mesh_reception/status"]
    visualizer = (visualizer or "rviz") if view else "none"
    viewer_arguments = ["viz:=" + visualizer]
    if visualizer == "rerun" and rerun_runtime is not None:
        viewer_arguments.append(f"rerun_runtime:={Path(rerun_runtime).expanduser().resolve()}")
    if estimated:
        viewer_arguments += [f"initial_map_source:={source}",
                            f"learned_depth_runtime:={Path(depth_runtime).expanduser().resolve()}",
                            f"learned_depth_backend:={backend}",
                            f"learned_voxel_size:={estimated_voxel}",
                            f"learned_depth_max_range_m:={estimated_range}"]
        registration.append("--estimated-depth")
        health.append("--estimated-depth")
        mesh_health.append("--estimated-depth")
    commands = [("relay", relay)]
    if mapping:
        relay.append("--head-only")
        _, scene_source, _ = scene_frames(name)
        commands.append(("scene", [python, str(EXAMPLES / "tools/field_teammate_scene.py"), "--name", name]))
        commands.append(("path", [python, str(EXAMPLES / "tools/field_teammate_path.py"),
            "--input-topic", f"/{name}/research/long_throw/pose",
            "--output-topic", f"/{name}/trajectory", "--output-frame", scene_source]))
        launch = "live_hybrid_mapping.launch.py" if estimated else "live_dense_mapping.launch.py"
        commands += [("mapping", ["ros2", "launch", "horus_field_mapping", launch,
            f"hololens_host:={host}", f"teammate_name:={name}", "session_name:=scan",
            f"mapping_profile:={mapping_profile}",
            f"mapping_memory_type:={mapping_memory}",
            f"voxel_size:={measured_voxel}",
            f"output_root:={directory}", f"calibration_directory:={directory / 'calibration'}",
            *viewer_arguments]),
            ("adapter", [python, str(EXAMPLES / "tools/nvblox_mesh_to_horus.py"),
                "--input-topic", "/horus_measured_mapper/mesh", "--output-topic", "/horus/field/mesh",
                "--blocks-per-axis", "8", "--presentation-frame", scene_source])]
        if estimated:
            commands.append(("estimated_adapter", [python, str(EXAMPLES / "tools/nvblox_mesh_to_horus.py"),
                "--input-topic", "/horus_hybrid_mapper/mesh", "--output-topic", "/horus/field/estimated_mesh",
                "--blocks-per-axis", "8", "--presentation-frame", scene_source]))
        if adaptive:
            source_topic = "/horus/field/estimated_mesh" if source == "estimated" else "/horus/field/mesh"
            commands.append(("adaptive", [python, "-m", "horus.mapping.adaptive_mesh_ros",
                "--input-topic", source_topic, "--output-topic", "/horus/field/adaptive_mesh",
                "--frame-id", scene_source, "--save-source", str(directory / "adaptive_original"),
                "--scene-status-topic", f"/{name}/scene/status"]))
    else:
        registration.append("--receive-only")
        health.append("--receive-only")
    if mapping:
        commands.append(("mesh_health", mesh_health))
    commands += [("registration", registration), ("health", health)]
    return commands


def exited_mappers(directory):
    directory = (Path(directory) / "scan").resolve()
    result = {}
    for mapper in ("horus_measured_mapper", "horus_hybrid_mapper"):
        path = directory / f"mapper_exit_{mapper}.json"
        if not path.is_file():
            continue
        record = json.loads(path.read_text())
        if (record.get("schema") != "horus.mapper_exit.v1" or
                record.get("session_directory") != str(directory) or
                record.get("mapper_node") != "/" + mapper or
                type(record.get("pid")) is not int or record["pid"] <= 0 or
                type(record.get("returncode")) is not int):
            raise RuntimeError(f"Invalid mapper exit record: {path}")
        result["/" + mapper] = record["returncode"]
    return result


def save_map(directory, estimated=False):
    # The session guard stores ownership; never save a different launch's mapper.
    metadata = Path(f"/tmp/horus_dense_mapping_domain_{os.environ.get('ROS_DOMAIN_ID', '20')}.json")
    if not metadata.is_file():
        return
    owner = json.loads(metadata.read_text())
    if Path(owner.get("session_directory", "")).resolve() != (directory / "scan").resolve():
        return
    print("Saving map meshes and restorable volumes before stopping...", flush=True)
    targets = [("final", "/horus_measured_mapper", "save.log")]
    if estimated:
        targets.append(("final_estimated", "/horus_hybrid_mapper", "save_estimated.log"))
    successful = True
    dead = exited_mappers(directory)
    for label, mapper, filename in targets:
        if mapper in dead:
            print(f"{mapper} exited ({dead[mapper]}); its unpublished native volume is unavailable. "
                  "Preserving the last received mesh checkpoint.", flush=True)
            successful = False
            continue
        with (directory / filename).open("wb") as log:
            try:
                result = subprocess.run(["ros2", "run", "horus_field_mapping", "save_mapping", "--label", label,
                    "--mapper-node", mapper, "--session-name", "scan", "--output-root", str(directory), "--timeout", "120"],
                    stdout=log, stderr=subprocess.STDOUT, timeout=300)
                successful &= result.returncode == 0
                print(f"{label}: saved." if result.returncode == 0 else f"Map save failed; see {directory / filename}", flush=True)
            except subprocess.TimeoutExpired:
                print(f"Map save timed out; see {directory / filename}", flush=True)
                successful = False
    return successful


def main():
    args = build_parser().parse_args()
    if not args.name or not args.name.isascii() or any(not (c.isalnum() or c == "_") for c in args.name) or not args.name[0].isalpha():
        raise SystemExit("Use a teammate name beginning with a letter, containing letters, digits and underscores.")
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        raise SystemExit("Duration must be finite and positive.")
    if args.receive_only and args.estimated_depth:
        raise SystemExit("Estimated-depth mapping cannot be combined with --receive-only.")
    if args.receive_only and args.map_source:
        raise SystemExit("Map source selection needs mapping, not --receive-only.")
    try:
        source = resolve_map_source(args.estimated_depth, args.map_source)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    if args.adaptive and (args.receive_only or args.mapping_profile != "quality"):
        raise SystemExit("Use --adaptive with mapping and the default quality profile; fine detail needs a 1 cm source.")
    if not math.isfinite(args.estimated_range) or not .25 < args.estimated_range <= 50:
        raise SystemExit("Estimated range must be finite and between 0.25 and 50 metres; distant estimates are not measurements.")
    sys.path.insert(0, str(CONNECTOR / "scripts"))
    visualizer = "none" if args.no_view or args.receive_only else args.visualizer
    local = preflight(not args.receive_only, visualizer, args.rerun_runtime,
                      args.estimated_depth, args.depth_runtime, args.depth_backend, args.mapping_memory)
    if args.adaptive:
        from horus.mapping.adaptive_mesh import ensure_native
        local["adaptive_native"] = str(ensure_native())
        local["adaptive_display"] = {"fine_m": .01, "context_m": .03,
            "source": source}
    if not args.receive_only:
        local["mapping_profile"] = args.mapping_profile
        local["mesh_render_profile"] = args.mesh_render_profile
        local["quest_map_source"] = source
        local["quest_map_topic"] = "/horus/field/adaptive_mesh" if args.adaptive else (
            "/horus/field/estimated_mesh" if source == "estimated" else "/horus/field/mesh")
        local["measured_voxel_size_m"] = PROFILE_VOXELS[args.mapping_profile][0]
        if args.estimated_depth:
            local["estimated_voxel_size_m"] = PROFILE_VOXELS[args.mapping_profile][1]
        if args.adaptive:
            local["measured_voxel_size_m"] = .01
            if args.estimated_depth:
                local["estimated_voxel_size_m"] = .01
        local["map_coverage"] = "complete observed map; no height/radius exclusions or map clearing"
    if args.check:
        if args.host:
            local["headset"] = probe_headset(str(ipaddress.IPv4Address(args.host)), not args.receive_only)
        print(json.dumps(local, indent=2))
        return 0
    host = args.host or os.environ.get("HORUS_FIELD_HOLOLENS_HOST")
    if not host and sys.stdin.isatty():
        host = input("HoloLens IP shown in HORUS Field: ").strip()
    if not host:
        raise SystemExit("Run ./field-demo <HoloLens_IP>, or ./field-demo --check for local checks.")
    host = str(ipaddress.IPv4Address(host))
    reject_existing_session(not args.receive_only, name=args.name)
    local["headset"] = probe_headset(host, not args.receive_only, verify_protocol=False)
    root = Path(os.environ.get("HORUS_FIELD_OUTPUT_ROOT", "~/horus_data/field_mapping/dense")).expanduser().resolve()
    directory = root / datetime.now(timezone.utc).strftime("field_%Y%m%dT%H%M%S_%fZ")
    directory.mkdir(parents=True)
    commands = command_plan(host, directory, args.name, not args.receive_only, not args.no_view,
                            visualizer, args.rerun_runtime, args.estimated_depth, args.depth_runtime,
                            args.depth_backend, args.estimated_range, args.mapping_profile, args.adaptive,
                            args.mapping_memory, source, args.mesh_render_profile)
    (directory / "launch.json").write_text(json.dumps({"local": local, "commands": commands,
        "data": "real headset only; no synthetic feeds", "voice": "not launched by this sensor test"}, indent=2) + "\n")
    children = Children(directory)
    print(f"Field test on ROS domain {os.environ.get('ROS_DOMAIN_ID')}; output: {directory}", flush=True)
    print("PC reception starts independently of Quest. For MR: connect HORUS to this PC, port10000, then create the workspace.", flush=True)
    print("Ctrl+C saves the measured map (when available) and stops only this launch.", flush=True)
    if not args.receive_only:
        print(f"Mapping profile: {args.mapping_profile}; measured voxels {local['measured_voxel_size_m'] * 100:g} cm; "
              "complete observed map, no height/radius exclusions.", flush=True)
        print(f"Map layer memory: {args.mapping_memory}; source resolution and coverage unchanged.", flush=True)
        print(f"Quest mesh render profile: {args.mesh_render_profile}; LOD approximates distant display geometry; "
              "SpaceWarp synthesizes frames. Source map and colours remain complete.", flush=True)
        print(f"HORUS map source: {source}{' (provisional completion)' if source == 'estimated' else ' (physical depth)'}; "
              f"topic={local['quest_map_topic']}. Uniform source delivery retains all geometry; "
              "only --adaptive requests server-generated coarse context.", flush=True)
        if args.estimated_depth:
            print(f"Separate provisional map: {local['estimated_voxel_size_m'] * 100:g} cm voxels.", flush=True)
    if args.estimated_depth:
        print(f"RGB-D completion: {args.depth_backend}; separate provisional layer, range limit={args.estimated_range}m (not measured range).", flush=True)
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    previous = signal.signal(signal.SIGTERM, interrupt)
    result = 0
    try:
        for name, command in commands:
            children.start(name, command, console=name == "health")
        started = time.monotonic()
        while args.duration is None or time.monotonic() - started < args.duration:
            dead = exited_mappers(directory)
            if dead:
                raise RuntimeError(f"Mapper exited unexpectedly: {dead}; preserving surviving maps and last mesh.")
            for name, process in children.processes:
                if process.poll() is not None:
                    raise RuntimeError(f"{name} exited ({process.returncode}); see {directory / (name + '.log')}")
            time.sleep(.5)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"Field test stopped: {error}", file=sys.stderr)
        result = 1
    finally:
        prior_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            if not args.receive_only:
                try:
                    if not save_map(directory, args.estimated_depth):
                        result = 1
                except (OSError, ValueError) as error:
                    print(f"Map save could not complete: {error}", file=sys.stderr)
                    result = 1
        finally:
            children.stop()
            signal.signal(signal.SIGTERM, previous)
            signal.signal(signal.SIGINT, prior_sigint)
        if args.adaptive:
            archive = directory / "adaptive_original/manifest.json"
            try:
                captured = json.loads(archive.read_text())
                verified = captured.get("input_inventory_verified") is True
                print(f"Original fine mesh: {archive.parent}; {captured['triangles']:,} triangles; "
                      f"source inventory {'verified' if verified else 'NOT verified; inspect before replay'}.", flush=True)
                if not verified:
                    result = 1
            except (OSError, ValueError, KeyError) as error:
                print(f"Original mesh archive was not verified: {error}; see adaptive.log", flush=True)
                result = 1
        rows = (directory / "reception.ndjson")
        validated = False
        if rows.is_file():
            # Keep only a bounded last line; this is not an experimental aggregate.
            with rows.open("rb") as stream:
                stream.seek(max(0, rows.stat().st_size - 128 * 1024))
                last = stream.read().splitlines()
            if last:
                validated = bool(json.loads(last[-1]).get("all_required_data_present"))
        if not validated:
            print("Not all requested sensor streams were validated. Logs are preserved; this is not a passed sensor test.", flush=True)
            if result == 0:
                result = 2
        (directory / "exit.json").write_text(json.dumps({"result": result, "data_validated_at_stop": validated,
            "children": {name: process.returncode for name, process in children.processes}}, indent=2) + "\n")
    return result


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ImportError, OSError, ValueError) as error:
        raise SystemExit(f"Field setup error: {error}")
