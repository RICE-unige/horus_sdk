#!/usr/bin/env python3
"""Validate real URDF TF trees in RViz using robot_state_publisher per robot.

This example is intentionally focused on TF + robot description correctness.
It launches robot_state_publisher instances (one per robot), publishes zeroed
joint states, and optionally publishes map->base transforms so both robots can
be viewed together in RViz.
"""

from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)


GO1_DEFAULT_URL_NOTE = "Go1 from local assets (or explicit --legged-urdf)."
JACKAL_DEFAULT_URL_NOTE = "Jackal from local assets (or explicit --wheeled-urdf)."


@dataclass
class RobotDescriptionSpec:
    name: str
    urdf_path: str
    base_frame: str
    offset_x: float
    offset_y: float


def _first_existing_path(candidates: Sequence[str]) -> str:
    for candidate in candidates:
        if candidate and os.path.isfile(candidate):
            return candidate
    return ""


def _local_asset_path(filename: str) -> str:
    return str(Path(SCRIPT_DIR) / ".local_assets" / "robot_descriptions" / filename)


def _resolve_demo_urdf(explicit: str, local_candidates: Sequence[str], fallback_candidates: Sequence[str]) -> str:
    if explicit:
        candidate = os.path.expandvars(os.path.expanduser(explicit.strip()))
        if os.path.isfile(candidate):
            return candidate
        return ""
    return _first_existing_path([*local_candidates, *fallback_candidates])


def _resolve_wheeled_urdf(explicit: str) -> str:
    return _resolve_demo_urdf(
        explicit=explicit,
        local_candidates=(
            _local_asset_path("jackal.urdf"),
        ),
        fallback_candidates=(
            "/mnt/c/Users/adeko/horus/Assets/URDF/nova_carter_fbx.urdf",
            "C:/Users/adeko/horus/Assets/URDF/nova_carter_fbx.urdf",
            _local_asset_path("jackal.urdf.xacro"),
        ),
    )


def _resolve_legged_urdf(explicit: str) -> str:
    return _resolve_demo_urdf(
        explicit=explicit,
        local_candidates=(
            _local_asset_path("go1.urdf"),
        ),
        fallback_candidates=(
            "/mnt/c/Users/adeko/newton/newton/examples/assets/quadruped.urdf",
            "C:/Users/adeko/newton/newton/examples/assets/quadruped.urdf",
        ),
    )


def _expand_xacro_if_needed(path: str) -> str:
    lower = path.lower()
    if not lower.endswith(".xacro"):
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read()

    commands: List[List[str]] = []
    xacro_exec = shutil.which("xacro")
    if xacro_exec:
        commands.append([xacro_exec, path])

    ros2_exec = shutil.which("ros2")
    if ros2_exec:
        commands.append([ros2_exec, "run", "xacro", "xacro", path])

    commands.append(["python3", "-m", "xacro", path])

    last_error = "xacro tool not found."
    for command in commands:
        try:
            proc = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=20.0,
                cwd=os.path.dirname(path) or None,
            )
        except Exception as exc:
            last_error = f"failed running {' '.join(command)}: {exc}"
            continue

        if proc.returncode != 0:
            stderr = str(proc.stderr or proc.stdout or "").strip()
            last_error = stderr or f"xacro failed ({' '.join(command)}), rc={proc.returncode}"
            continue

        payload = str(proc.stdout or "")
        if "<robot" in payload:
            return payload

        last_error = f"xacro output missing <robot> tag ({' '.join(command)})"

    ros_distro = os.environ.get("ROS_DISTRO", "").strip()
    distro_hint = f"ros-{ros_distro}-xacro" if ros_distro else "ros-<distro>-xacro"
    raise RuntimeError(
        f"Could not expand xacro file '{path}'. {last_error} "
        f"Install xacro (`sudo apt install {distro_hint}`) or provide a resolved .urdf."
    )


def _extract_movable_joint_names(urdf_xml: str) -> List[str]:
    root = ET.fromstring(urdf_xml)
    joint_names: List[str] = []
    for joint_el in root.findall("joint"):
        name = str(joint_el.attrib.get("name", "")).strip()
        if not name:
            continue
        joint_type = str(joint_el.attrib.get("type", "fixed")).strip().lower()
        if joint_type == "fixed":
            continue
        joint_names.append(name)
    return joint_names


def _extract_link_names(urdf_xml: str) -> List[str]:
    root = ET.fromstring(urdf_xml)
    names: List[str] = []
    for link_el in root.findall("link"):
        name = str(link_el.attrib.get("name", "")).strip()
        if name:
            names.append(name)
    return names


def _find_root_link_name(urdf_xml: str) -> str:
    root = ET.fromstring(urdf_xml)
    links: List[str] = []
    child_links: set[str] = set()
    for link_el in root.findall("link"):
        name = str(link_el.attrib.get("name", "")).strip()
        if name:
            links.append(name)

    for joint_el in root.findall("joint"):
        child_el = joint_el.find("child")
        if child_el is None:
            continue
        child_name = str(child_el.attrib.get("link", "")).strip()
        if child_name:
            child_links.add(child_name)

    for link_name in links:
        if link_name not in child_links:
            return link_name
    return links[0] if links else "base_link"


def _write_temp_urdf(robot_name: str, urdf_xml: str) -> Tuple[str, str]:
    temp_dir = tempfile.mkdtemp(prefix=f"horus_{robot_name}_urdf_")
    urdf_path = os.path.join(temp_dir, f"{robot_name}.urdf")
    with open(urdf_path, "w", encoding="utf-8") as handle:
        handle.write(urdf_xml)
    return temp_dir, urdf_path


class RobotDescriptionRvizValidationNode(Node):
    def __init__(
        self,
        specs: Sequence[RobotDescriptionSpec],
        map_frame: str,
        publish_map_tf: bool,
        root_tf_rate_hz: float,
        joint_state_rate_hz: float,
        description_republish_hz: float,
    ):
        super().__init__("robot_description_rviz_validation_demo")

        self._specs = list(specs)
        self._map_frame = map_frame
        self._publish_map_tf = publish_map_tf
        self._description_xml_by_robot: Dict[str, str] = {}
        self._joint_names_by_robot: Dict[str, List[str]] = {}
        self._joint_publishers = {}
        self._description_publishers = {}
        self._rsp_processes = []
        self._temp_dirs: List[str] = []

        tf_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        joint_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._tf_pub = self.create_publisher(TFMessage, "/tf", tf_qos)

        for spec in self._specs:
            urdf_xml = _expand_xacro_if_needed(spec.urdf_path)
            link_names = set(_extract_link_names(urdf_xml))
            if spec.base_frame not in link_names:
                resolved_root = _find_root_link_name(urdf_xml)
                self.get_logger().warning(
                    f"[RobotDescription] base_frame '{spec.base_frame}' not found in URDF for '{spec.name}'. "
                    f"Using root link '{resolved_root}' for map connection."
                )
                spec.base_frame = resolved_root

            self._description_xml_by_robot[spec.name] = urdf_xml
            self._joint_names_by_robot[spec.name] = _extract_movable_joint_names(urdf_xml)
            self._joint_publishers[spec.name] = self.create_publisher(
                JointState, f"/{spec.name}/joint_states", joint_qos
            )
            self._description_publishers[spec.name] = self.create_publisher(
                String, f"/{spec.name}/robot_description", description_qos
            )
            self._launch_robot_state_publisher(spec, urdf_xml)

        self._publish_descriptions_once()

        if self._publish_map_tf:
            period = 1.0 / max(1.0, float(root_tf_rate_hz))
            self.create_timer(period, self._publish_root_tf)

        joint_period = 1.0 / max(1.0, float(joint_state_rate_hz))
        self.create_timer(joint_period, self._publish_joint_states)

        if description_republish_hz > 0.0:
            description_period = 1.0 / float(description_republish_hz)
            self.create_timer(description_period, self._publish_descriptions_once)

    def _launch_robot_state_publisher(self, spec: RobotDescriptionSpec, urdf_xml: str) -> None:
        temp_dir, temp_urdf_path = _write_temp_urdf(spec.name, urdf_xml)
        self._temp_dirs.append(temp_dir)

        command = [
            "ros2",
            "run",
            "robot_state_publisher",
            "robot_state_publisher",
            temp_urdf_path,
            "--ros-args",
            "-p",
            f"frame_prefix:={spec.name}/",
            "-p",
            "publish_frequency:=60.0",
            "-r",
            f"/joint_states:=/{spec.name}/joint_states",
        ]

        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self._rsp_processes.append(process)
        self.get_logger().info(
            f"[RobotDescription] started robot_state_publisher for '{spec.name}' "
            f"(pid={process.pid}, base_frame={spec.base_frame}, urdf='{spec.urdf_path}')."
        )

    def _publish_descriptions_once(self) -> None:
        for spec in self._specs:
            payload = self._description_xml_by_robot.get(spec.name, "")
            if not payload:
                continue
            msg = String()
            msg.data = payload
            self._description_publishers[spec.name].publish(msg)

    def _publish_root_tf(self) -> None:
        now = self.get_clock().now().to_msg()
        transforms: List[TransformStamped] = []
        for spec in self._specs:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self._map_frame
            tf.child_frame_id = f"{spec.name}/{spec.base_frame}"
            tf.transform.translation.x = float(spec.offset_x)
            tf.transform.translation.y = float(spec.offset_y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = 0.0
            tf.transform.rotation.w = 1.0
            transforms.append(tf)

        if transforms:
            self._tf_pub.publish(TFMessage(transforms=transforms))

    def _publish_joint_states(self) -> None:
        now = self.get_clock().now().to_msg()
        for spec in self._specs:
            names = self._joint_names_by_robot.get(spec.name, [])
            msg = JointState()
            msg.header.stamp = now
            msg.name = list(names)
            msg.position = [0.0] * len(names)
            self._joint_publishers[spec.name].publish(msg)

    def shutdown(self) -> None:
        for proc in self._rsp_processes:
            if proc is None or proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception:
                pass

        end_time = time.time() + 2.0
        for proc in self._rsp_processes:
            if proc is None:
                continue
            if proc.poll() is not None:
                continue
            remaining = max(0.0, end_time - time.time())
            try:
                proc.wait(timeout=remaining)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass

        for temp_dir in self._temp_dirs:
            shutil.rmtree(temp_dir, ignore_errors=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "RViz validation for Robot Description V1. "
            "Launches robot_state_publisher per robot from real URDF and publishes zeroed joint states."
        )
    )
    parser.add_argument("--wheeled-name", default="jackal", help="Wheeled robot name prefix.")
    parser.add_argument("--legged-name", default="go1", help="Legged robot name prefix.")
    parser.add_argument(
        "--wheeled-urdf",
        default="",
        help=f"Wheeled URDF/xacro path. Default resolution: {JACKAL_DEFAULT_URL_NOTE}",
    )
    parser.add_argument(
        "--legged-urdf",
        default="",
        help=f"Legged URDF/xacro path. Default resolution: {GO1_DEFAULT_URL_NOTE}",
    )
    parser.add_argument("--wheeled-base-frame", default="base_link", help="Wheeled robot base frame name.")
    parser.add_argument("--legged-base-frame", default="base", help="Legged robot base frame name.")
    parser.add_argument("--map-frame", default="map", help="Global fixed frame for root transforms.")
    parser.add_argument(
        "--robot-separation-m",
        type=float,
        default=1.8,
        help="X-axis separation between wheeled and legged robot roots in map frame.",
    )
    parser.add_argument(
        "--publish-map-tf",
        dest="publish_map_tf",
        action="store_true",
        default=True,
        help="Publish map-><robot>/<base_frame> static root TF (default: on).",
    )
    parser.add_argument(
        "--no-publish-map-tf",
        dest="publish_map_tf",
        action="store_false",
        help="Do not publish map root TF (only URDF tree from each base frame).",
    )
    parser.add_argument("--root-tf-rate", type=float, default=20.0, help="Root TF publish rate (Hz).")
    parser.add_argument("--joint-state-rate", type=float, default=30.0, help="JointState publish rate (Hz).")
    parser.add_argument(
        "--description-republish-rate",
        type=float,
        default=0.5,
        help="Re-publish /<robot>/robot_description at this rate (Hz). Set 0 to publish once.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()

    wheeled_urdf = _resolve_wheeled_urdf(args.wheeled_urdf)
    legged_urdf = _resolve_legged_urdf(args.legged_urdf)

    missing: List[str] = []
    if not wheeled_urdf or not os.path.isfile(wheeled_urdf):
        missing.append(f"wheeled URDF unresolved ({args.wheeled_urdf or 'default lookup'})")
    if not legged_urdf or not os.path.isfile(legged_urdf):
        missing.append(f"legged URDF unresolved ({args.legged_urdf or 'default lookup'})")
    if missing:
        print("ERROR: Robot description demo URDF path missing/unreadable:")
        for item in missing:
            print(f"  - {item}")
        print("Tip: fetch local assets first:")
        print("  python3 python/examples/tools/fetch_robot_description_assets.py")
        return

    separation = max(0.2, float(args.robot_separation_m))
    specs = [
        RobotDescriptionSpec(
            name=str(args.wheeled_name).strip(),
            urdf_path=wheeled_urdf,
            base_frame=str(args.wheeled_base_frame).strip() or "base_link",
            offset_x=-0.5 * separation,
            offset_y=0.0,
        ),
        RobotDescriptionSpec(
            name=str(args.legged_name).strip(),
            urdf_path=legged_urdf,
            base_frame=str(args.legged_base_frame).strip() or "base_link",
            offset_x=0.5 * separation,
            offset_y=0.0,
        ),
    ]

    print("[robot-description-rviz] wheeled:", specs[0].name, "->", specs[0].urdf_path)
    print("[robot-description-rviz] legged :", specs[1].name, "->", specs[1].urdf_path)
    print(
        "[robot-description-rviz] publish_map_tf=",
        bool(args.publish_map_tf),
        "map_frame=",
        args.map_frame,
        "separation_m=",
        separation,
    )

    rclpy.init()
    node = None
    try:
        node = RobotDescriptionRvizValidationNode(
            specs=specs,
            map_frame=str(args.map_frame).strip() or "map",
            publish_map_tf=bool(args.publish_map_tf),
            root_tf_rate_hz=float(args.root_tf_rate),
            joint_state_rate_hz=float(args.joint_state_rate),
            description_republish_hz=float(args.description_republish_rate),
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
