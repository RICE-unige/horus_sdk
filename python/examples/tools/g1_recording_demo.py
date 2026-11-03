"""Launch the complete recorded G1 workspace without a live robot or ROS setup steps."""

import argparse
from contextlib import ExitStack
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

from g1_recording_registration import ROBOT, TOPIC, URDF, make_registration
from mapping_visualization_demo import stop_owned
from tools.g1_recording_data import bag_path, load_occupancy, occupancy_path


def check(dataset="later", map_mode="live"):
    if dataset == "first" and map_mode == "live":
        raise ValueError("The first recording has no saved incremental mesh stream. Use ./g1-recording-demo first --map static.")
    import yaml
    from ament_index_python.packages import get_package_prefix
    from horus.description.robot_description_resolver import RobotDescriptionResolver
    from tools.g1_recording_replay import inspect

    for module in ("rclpy", "rosbag2_py", "nvblox_msgs", "cv2", "numpy"):
        if importlib.util.find_spec(module) is None:
            raise RuntimeError(f"Missing {module}; source the HORUS ROS workspace.")
    get_package_prefix("robot_state_publisher")
    get_package_prefix("horus_unity_bridge")
    bag = bag_path(dataset)
    if not URDF.is_file():
        raise FileNotFoundError(f"The recorded Dex3 G1 model is missing: {URDF}")
    _, info = inspect(bag, URDF, dataset, map_mode)
    grid = load_occupancy(occupancy_path(dataset))
    info["occupancy"] = True
    info["occupancy_cells"] = [grid.info.width, grid.info.height]
    if map_mode == "static":
        from tools.g1_completed_maps import mesh_arrays
        vertices, _, faces = mesh_arrays(dataset)
        info.update(map_vertices=len(vertices), map_triangles=len(faces))
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    info["duration_s"] = metadata["duration"]["nanoseconds"] / 1e9
    robot, viz = make_registration(info)
    resolver = RobotDescriptionResolver()
    asset = resolver.resolve_for_robot(robot)
    if asset is None:
        raise RuntimeError(f"G1 model resolution failed: {resolver.last_error}")
    visuals = asset.payload_dict.get("visual_links", [])
    mesh_ids = {mesh["mesh_id"] for mesh in asset.payload_dict.get("mesh_assets", [])}
    missing = [visual["name"] for visual in visuals if visual.get("mesh_id") not in mesh_ids]
    if missing or not asset.manifest.supports_visual_meshes:
        raise RuntimeError(f"Incomplete G1 mesh payload: {missing}")
    info.update(mesh_assets=asset.manifest.mesh_asset_count,
                model_triangles=sum(m["triangle_count"] for m in asset.payload_dict.get("mesh_assets", [])))
    # Fixed URDF visuals are merged into their moving parent by the baker.
    # Compare source triangle totals instead of requiring a renderer per link.
    source_counts = {}
    total = 0
    for mesh in ET.parse(URDF).getroot().findall("link/visual/geometry/mesh"):
        uri = mesh.attrib["filename"]
        prefix = "package://unitree_g1_description/"
        if not uri.startswith(prefix):
            raise ValueError(f"Unexpected G1 mesh URI: {uri}")
        source = URDF.parent.parent / uri[len(prefix):]
        if source not in source_counts:
            # This recorded model ships binary STL assets. Check the standard
            # facet count and file length without loading every vertex twice.
            with source.open("rb") as stream:
                header = stream.read(84)
            count = struct.unpack_from("<I", header, 80)[0] if len(header) == 84 else -1
            if source.stat().st_size != 84 + 50 * count:
                raise ValueError(f"Unexpected non-binary STL asset: {source}")
            source_counts[source] = count
        total += source_counts[source]
    if total != info["model_triangles"]:
        raise RuntimeError(f"G1 mesh was reduced or incomplete: {info['model_triangles']}/{total} triangles")
    print(f"Recorded G1: {info['joint_count']} joints, {info['mesh_assets']} mesh assets, "
          f"{info['model_triangles']:,} triangles.\nRGB: {info['width']}x{info['height']}; "
          f"walk: {info['duration_s']:.1f}s; LiDAR + walked path + colored nvblox ({map_mode}).\n"
          f"Occupancy: saved completed grid, {grid.info.width}x{grid.info.height}, {grid.info.resolution:g}m/cell.\n"
          f"Source: {bag}\nROS domain: {os.environ.get('ROS_DOMAIN_ID', '0')}; control disabled.", flush=True)
    if map_mode == "static":
        print(f"Completed map: {info['map_vertices']:,} source vertices, "
              f"{info['map_triangles']:,} faces; original RGB retained.", flush=True)
    return bag, info


def request_start():
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node("horus_g1_replay_start")
    try:
        client = node.create_client(Trigger, TOPIC + "/start")
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError("G1 replay is not running in this ROS domain. Launch ./g1-recording-demo first.")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError("Replay start request timed out.")
        reply = future.result()
        print(reply.message, flush=True)
        return reply.success
    finally:
        node.destroy_node()
        rclpy.shutdown()


def ensure_port_free():
    with socket.socket() as sock:
        sock.settimeout(0.3)
        if sock.connect_ex(("127.0.0.1", 10000)) == 0:
            raise RuntimeError("A HORUS bridge is already running on port 10000. Stop the old demo "
                               "with Ctrl+C before starting this one. Nothing has been stopped automatically.")


def main(argv=None):
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", nargs="?", choices=("first", "later"), default="later")
    parser.add_argument("--map", dest="map_mode", choices=("static", "live"),
                        help="static loads the completed map; live replays archived mesh updates (later only).")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--check", action="store_true", help="Validate recording and compile the robot model without starting ROS nodes.")
    group.add_argument("--start", action="store_true", help="Start the waiting walk, also usable from a second terminal.")
    args = parser.parse_args(argv)
    if args.start:
        return 0 if request_start() else 1
    map_mode = args.map_mode or ("static" if args.dataset == "first" else "live")
    bag, info = check(args.dataset, map_mode)
    if args.check:
        return 0
    ensure_port_free()
    root = Path.home() / ".cache/horus/g1-recording-demo"
    root.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="replay-", dir=root))
    info_file, ready = run / "camera-info.json", run / "ready.json"
    maps_ready = run / "maps-ready.json"
    info_file.write_text(json.dumps(info, indent=2) + "\n")
    params = run / "robot-state-publisher.yaml"
    params.write_text(yaml.safe_dump({"/**": {"ros__parameters": {
        "robot_description": URDF.read_text(), "frame_prefix": ROBOT + "/",
        "use_sim_time": False, "publish_frequency": 50.0,
    }}}))
    commands = [
        ["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args",
         "--params-file", str(params), "-r", "joint_states:=" + TOPIC + "/joint_states"],
        [sys.executable, "-m", "tools.g1_completed_maps", "--dataset", args.dataset,
         "--map", map_mode, "--ready-file", str(maps_ready)],
        [sys.executable, "-m", "tools.g1_recording_replay", "--bag", str(bag), "--urdf", str(URDF),
         "--ready-file", str(ready), "--dataset", args.dataset, "--map", map_mode],
    ]
    if map_mode == "live":
        commands.insert(1, [sys.executable, "-m", "tools.nvblox_mesh_to_horus",
            "--input-topic", TOPIC + "/mesh/source", "--output-topic", TOPIC + "/mesh",
            "--blocks-per-axis", "4", "--color-mode", "source"])
    processes = []
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    print(f"Logs: {run}\nConnect HORUS to this PC on port 10000 and create a workspace.\n"
          "Wait for the G1 model, then press Enter here to start the walk.\n"
          "Alternatively: ./g1-recording-demo --start in a second terminal.\n"
          "The last scene stays visible. Ctrl+C stops this demo only.", flush=True)
    with ExitStack() as stack:
        try:
            for index, command in enumerate(commands):
                log = stack.enter_context((run / f"source-{index}.log").open("w"))
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            deadline = time.monotonic() + 30
            while not (ready.is_file() and maps_ready.is_file()):
                if any(p.poll() is not None for p in processes) or time.monotonic() > deadline:
                    raise RuntimeError(f"G1 source did not become ready; see {run}")
                time.sleep(0.1)
            processes.append(subprocess.Popen([sys.executable, "-m", "g1_recording_registration",
                "--camera-info", str(info_file)], start_new_session=True))
            interactive = sys.stdin.isatty()
            while True:
                for p in processes:
                    if p.poll() is not None:
                        if p is processes[-1] and p.returncode == 0:
                            return 0
                        raise RuntimeError(f"Demo child exited ({p.returncode}); see {run}")
                if interactive and select.select([sys.stdin], [], [], 0.2)[0]:
                    if sys.stdin.readline() == "":
                        interactive = False
                    else:
                        request_start()
                elif not interactive:
                    time.sleep(0.2)
        except KeyboardInterrupt:
            return 0
        finally:
            stop_owned(processes)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError) as exc:
        raise SystemExit(str(exc))
