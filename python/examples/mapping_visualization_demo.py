#!/usr/bin/env python3
"""One isolated workspace map at a time. No robot, camera or experiment workloads."""

import argparse
from contextlib import ExitStack
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
TOOLS = HERE / "tools"
TOPIC = "/horus/mapping_demo/map"
MODES = ("mesh", "indexed", "points", "points-refresh", "nvblox", "octomap",
         "voxblox-marker", "gaussian", "gaussian-points", "gaussian-no-covariance", "gaussian-mono",
         "remote-diagnostic", "remote-mesh", "remote-points", "remote-gaussian")
SPLAT = Path.home() / ".cache/horus/gaussian_splatting/runtime/playroom_30000_50000_5a34144cf4cf82e3.ply"


def require_webrtc_bridge(capability_file=None):
    try:
        if capability_file is None:
            from ament_index_python.packages import get_package_share_directory
            capability_file = Path(get_package_share_directory("horus_unity_bridge")) / "build_capabilities.json"
        capabilities = json.loads(Path(capability_file).read_text())
        if capabilities.get("schema") == "horus.bridge.build.v1" and capabilities.get("webrtc") is True:
            return
    except (ImportError, LookupError, OSError, ValueError, AttributeError):
        pass
    raise RuntimeError(
        "The selected local HORUS bridge has no verified WebRTC build. Rebuild horus_unity_bridge "
        "with -DENABLE_WEBRTC=ON (and -DHORUS_ALLOW_LIBDATACHANNEL_FETCH=ON if needed), "
        "source that workspace and stop an older running bridge normally. "
        "See python/examples/MAPPING_VISUALIZATION_TESTING.md."
    )


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", choices=MODES)
    parser.add_argument("--check", action="store_true", help="Check dependencies without starting registration.")
    parser.add_argument("--workspace-scale", type=float, default=0.05)
    parser.add_argument("--ply", type=Path, help="Gaussian input; every splat in this file is sent.")
    parser.add_argument("--dataset", choices=("g1", "spot", "cow-lady", "freiburg"),
                        help="Real capture to display; defaults to G1 mesh/replay, Cow and Lady points, Freiburg OctoMap.")
    parser.add_argument("--synthetic", action="store_true", help="Developer-only generated regression fixture, never the default.")
    parser.add_argument("--publish-only", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--register-only", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--ready-file", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not 0.001 <= args.workspace_scale <= 10:
        parser.error("workspace scale must be between 0.001 and 10")
    if args.synthetic and (args.dataset or args.path.startswith("remote-")):
        parser.error("--synthetic is only for native fixture regression tests")
    if args.dataset and "gaussian" in args.path:
        parser.error("Gaussian datasets are selected with --ply")
    if args.ply and "gaussian" not in args.path:
        parser.error("--ply is only supported for Gaussian modes")
    return args


def dataset_for(args):
    return args.dataset or ("freiburg" if args.path == "octomap" else
                           "cow-lady" if args.path in ("points", "remote-points") else "g1")


def real_record(args):
    from tools.real_mapping_data import record
    kind = "bag" if args.path in ("nvblox", "points-refresh") else "octomap" if args.path == "octomap" else (
        "points" if args.path in ("points", "remote-points") else "mesh")
    return record(dataset_for(args), kind)


def preflight(args):
    modules = ["rclpy", "numpy", "sensor_msgs", "visualization_msgs", "horus"]
    if args.path == "nvblox":
        modules.append("nvblox_msgs")
    if args.path in ("nvblox", "points-refresh") and not args.synthetic:
        modules.append("rosbag2_py")
    if args.path == "octomap":
        modules.append("octomap_msgs")
    if args.path.startswith("gaussian"):
        path = (args.ply or SPLAT).expanduser()
        if not path.is_file():
            raise RuntimeError(f"Missing Gaussian fixture {path}. Select an existing 3DGS PLY with --ply.")
        from tools.publish_gaussian_splat_fixture import read_ply_header
        count = read_ply_header(path).vertex_count
        if not 0 < count <= 2_000_000:
            raise RuntimeError(f"{count} splats exceed the native demo admission limit; input will not be truncated.")
        print(f"Source PLY: {path} ({count} splats, no new subset or SH pruning)", flush=True)
    if args.path.startswith("remote-"):
        modules.extend(("torch", "cv2"))
        if args.path in ("remote-mesh", "remote-diagnostic"):
            modules.append("nvdiffrast")
        if args.path == "remote-gaussian":
            modules.extend(("gsplat", "plyfile"))
    missing = [name for name in modules if importlib.util.find_spec(name) is None]
    if missing:
        raise RuntimeError(f"Missing dependencies in {sys.executable}: {', '.join(missing)}")
    if not args.synthetic and "gaussian" not in args.path:
        entry = real_record(args)
        print(f"Real source: {entry['path']}\nProvenance: {entry['provenance']}", flush=True)
    if args.path.startswith("remote-"):
        import torch
        if not torch.cuda.is_available():
            raise RuntimeError("Remote rendering requires an available CUDA GPU")
        if args.path != "remote-gaussian":
            from tools.real_mapping_data import arrays
            source = Path(real_record(args)["path"])
            arrays(source, mesh=args.path != "remote-points")
        else:
            from horus.remote_rendering.gaussian_renderer import resolve_playroom_splat
            source = resolve_playroom_splat(args.ply)
        print(f"Remote source: {source}; no source voxel reduction", flush=True)
        if args.path != "remote-diagnostic":
            require_webrtc_bridge()
            from horus.remote_rendering.nvenc_encoder import _load_low_latency_codec
            _load_low_latency_codec()
    print(f"Ready: {args.path}; ROS domain {os.environ.get('ROS_DOMAIN_ID', '0')}; local ROS only.", flush=True)


def registration(args):
    from horus.robot import WorkspaceVisualization
    resource = WorkspaceVisualization("mapping_demo")
    viz = resource.create_dataviz("mapping_demo")
    mesh = {"max_triangles": 2_000_000, "use_vertex_colors": True,
            "double_sided": False, "source_coordinate_space": "enu",
            "static_mesh": True, "use_indexed_binary": args.path == "indexed",
            "max_chunk_renderers": 1024, "enable_perf_diagnostics": True}
    if args.path == "nvblox":
        viz.add_nvblox_mesh(TOPIC)
    elif args.path == "octomap":
        viz.add_3d_octomap(TOPIC, render_options={"max_triangles": 3_000_000,
            "use_vertex_colors": True, "alpha": 1.0, "static_mesh": True}, transport_lane="bulk_strict")
    elif args.path.startswith("points"):
        viz.add_3d_map(TOPIC, update_mode="refresh" if args.path == "points-refresh" else "static",
            render_options={"render_all_points": True, "max_points_per_frame": 0,
                "base_sample_stride": 1, "enable_adaptive_quality": False,
                "enable_view_frustum_culling": True, "enable_subpixel_culling": False,
                "point_size": 0.06, "point_shape": "circle", "render_mode": "opaque_fast",
                "enable_perf_diagnostics": True})
    elif args.path.startswith("gaussian"):
        mode = {"gaussian": "splats", "gaussian-points": "debug_points",
                "gaussian-no-covariance": "no_covariance", "gaussian-mono": "mono_center_eye"}[args.path]
        viz.add_gaussian_splat_map(render_options={"preserve_source_detail": True,
            "binary_chunks": True,
            "max_splats": 2_000_000, "render_mode": mode, "pointcloud_fallback": False})
    else:
        viz.add_3d_mesh(TOPIC, render_options=mesh, transport_lane="bulk_strict")
    return resource, viz


def remote_command(args):
    scene = {"remote-diagnostic": "recorded_mesh", "remote-mesh": "recorded_mesh",
             "remote-points": "recorded_points", "remote-gaussian": "gaussian_playroom"}[args.path]
    command = [sys.executable, str(HERE / "remote_map_portal_registration.py"),
        "--scene", scene, "--protocol", "v3", "--profile", "balanced",
        "--transport", "ros" if args.path == "remote-diagnostic" else "webrtc",
        "--workspace-scale", str(args.workspace_scale), "--source-voxel-size", "0"]
    if args.path != "remote-gaussian":
        command += ["--dataset-path", real_record(args)["path"]]
    elif args.ply:
        command += ["--dataset-path", str(args.ply.expanduser())]
    return command


def child_commands(args, ready):
    source = [sys.executable, str(Path(__file__).resolve()), args.path, "--publish-only", "--ready-file", str(ready)]
    if args.synthetic:
        source += ["--synthetic"]
    else:
        if args.path in ("nvblox", "points-refresh"):
            source = [sys.executable, str(TOOLS / "replay_real_mapping_bag.py"), args.path,
                      "--topic", TOPIC + ("/source" if args.path == "nvblox" else ""),
                      "--ready-file", str(ready)]
        if not args.path.startswith("gaussian"):
            source += ["--dataset", dataset_for(args)]
    if args.path.startswith("gaussian"):
        source = [sys.executable, str(TOOLS / "publish_gaussian_splat_fixture.py"),
            "--ply", str((args.ply or SPLAT).expanduser()), "--splat-render-count", "0",
            "--sample-count", "1000", "--no-fake-robots", "--binary-chunks",
            "--chunk-republish-period", "30"]
    commands = [source]
    if args.path == "nvblox":
        commands.append([sys.executable, str(TOOLS / "nvblox_mesh_to_horus.py"),
            "--input-topic", TOPIC + "/source", "--output-topic", TOPIC, "--blocks-per-axis", "4",
            "--color-mode", "source"])
    elif args.path == "octomap":
        commands.append([sys.executable, str(TOOLS / "uav_sim_octomap_marker_relay.py"),
            "--input-topic", TOPIC + "/source", "--output-topic", TOPIC, "--alpha", "1"])
    elif args.path == "voxblox-marker":
        commands.append([sys.executable, str(TOOLS / "voxblox_mesh_to_horus_marker.py"),
            "--ros-api", "ros2", "--input-type", "marker", "--input-topic", TOPIC + "/source",
            "--output-topic", TOPIC, "--transient-local"])
    return commands


def stop_owned(processes):
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        for process in processes:
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        for process in processes:
            try:
                process.wait(timeout=max(0.01, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
        if sig == signal.SIGKILL:
            for process in processes:
                process.wait(timeout=2)


def main(argv=None):
    args = arguments(argv)
    if args.publish_only:
        from tools.mapping_demo_source import publish
        publish(args.path, TOPIC, args.ready_file, None if args.synthetic else dataset_for(args))
        return 0
    if args.register_only:
        from horus.robot import register_entities, is_registration_cancelled
        entity, viz = registration(args)
        ok, result = register_entities([entity], datavizs=[viz], workspace_scale=args.workspace_scale,
            compass_enabled=False, keep_alive=True)
        return 0 if ok or is_registration_cancelled(result) else 1
    preflight(args)
    if args.check:
        return 0
    if args.path.startswith("remote-"):
        os.execv(sys.executable, remote_command(args))
    log_root = Path.home() / ".cache/horus/mapping-demo"
    log_root.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix=args.path + "-", dir=log_root))
    ready = run / "source-ready.json"
    print(f"Logs: {run}\nConnect HORUS to this PC on port 10000, create/accept a workspace.\n"
          "No time limit. Ctrl+C stops only this demo's child processes.", flush=True)
    processes = []
    def request_stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, request_stop)
    with ExitStack() as stack:
        try:
            for index, command in enumerate(child_commands(args, ready)):
                log = stack.enter_context((run / f"source-{index}.log").open("w"))
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            registration_command = [sys.executable, str(Path(__file__).resolve()), args.path,
                "--register-only", "--workspace-scale", str(args.workspace_scale)]
            processes.append(subprocess.Popen(registration_command, start_new_session=True))
            while True:
                for process in processes:
                    code = process.poll()
                    if code is not None:
                        if process is not processes[-1] or code:
                            raise RuntimeError(f"Demo child {process.args} exited with {code}; see {run}")
                        return 0
                if ready.exists():
                    print("Source ready: " + ready.read_text().strip(), flush=True)
                    ready.unlink()
                time.sleep(0.25)
        except KeyboardInterrupt:
            return 0
        finally:
            stop_owned(processes)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ImportError, OSError, ValueError) as exc:
        raise SystemExit(f"Mapping demo: {exc}")
