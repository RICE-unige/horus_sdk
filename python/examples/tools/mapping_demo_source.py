"""Deterministic, colored mapping fixtures; never publishes robot commands."""

from array import array
import json
from pathlib import Path
import subprocess

import numpy as np


def scene_geometry():
    from horus.remote_rendering.synthetic_scene import build_industrial_site
    vertices, faces, colors = build_industrial_site().arrays()
    # The existing fixture is Unity-local. Convert to ROS FLU, floor at z=0.
    points = vertices[:, (2, 0, 1)].copy()
    points[:, 1] *= -1
    points[:, 2] -= points[:, 2].min()
    return points, faces[:, (0, 2, 1)], colors.astype(np.uint8)


def surface_points(count=100_000):
    vertices, faces, colors = scene_geometry()
    triangles = vertices[faces]
    areas = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                    triangles[:, 2] - triangles[:, 0]), axis=1)
    rng = np.random.default_rng(20261009)
    selected = rng.choice(len(triangles), count, p=areas / areas.sum())
    uv = rng.random((count, 2))
    uv[uv.sum(axis=1) > 1] = 1 - uv[uv.sum(axis=1) > 1]
    tri = triangles[selected]
    positions = tri[:, 0] + uv[:, :1] * (tri[:, 1] - tri[:, 0]) + uv[:, 1:] * (tri[:, 2] - tri[:, 0])
    return positions.astype(np.float32), colors[selected]


def octomap_bytes():
    source = Path(__file__).with_name("mapping_demo_octomap.cpp")
    root = Path.home() / ".cache/horus/mapping-demo"
    root.mkdir(parents=True, exist_ok=True)
    executable = root / "octomap-fixture"
    if not executable.exists() or executable.stat().st_mtime < source.stat().st_mtime:
        subprocess.run(["g++", "-std=c++17", "-O2", str(source), "-loctomap", "-loctomath",
                        "-o", str(executable)], check=True, timeout=90)
    return subprocess.check_output([str(executable)], timeout=30)


def publish(mode, topic, ready_file=None, dataset=None):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import Point, Point32
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import ColorRGBA, UInt8MultiArray
    from visualization_msgs.msg import Marker
    from tools.ply_to_horus_mesh_marker import build_indexed_binary_payload

    rclpy.init()
    node = rclpy.create_node("horus_mapping_demo_source")
    qos = QoSProfile(depth=1 if mode.startswith("points") or mode == "octomap" else 1024,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    source_topic = topic + "/source" if mode in ("nvblox", "octomap", "voxblox-marker") else topic
    record = None
    if dataset:
        from tools.real_mapping_data import arrays, record as get_record, octomap_file
        kind = "points" if mode.startswith("points") else "octomap" if mode == "octomap" else "mesh"
        record = get_record(dataset, kind)
        if kind == "mesh":
            vertices, rgb, faces = arrays(record["path"], mesh=True)
            positions = vertices[faces].reshape(-1, 3)
            colors = np.column_stack((rgb[faces].reshape(-1, 3) / 255, np.ones(len(positions))))
        elif kind == "points":
            xyz, point_rgb, _ = arrays(record["path"])
    else:
        vertices, faces, rgb = scene_geometry()
        positions = vertices[faces].reshape(-1, 3)
        colors = np.column_stack((np.repeat(rgb, 3, axis=0) / 255, np.ones(len(positions))))
    messages = []
    phase = 0
    if mode.startswith("points"):
        if not dataset:
            xyz, point_rgb = surface_points()
        packed = np.empty(len(xyz), dtype=[("xyz", "<f4", 3), ("rgba", "u1", 4)])
        packed["xyz"] = xyz
        packed["rgba"][:, :3] = point_rgb
        packed["rgba"][:, 3] = 255
        message = PointCloud2(height=1, width=len(xyz), point_step=16, row_step=len(xyz) * 16,
            is_dense=True, is_bigendian=False, data=packed.tobytes(), fields=[
                PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                for i, name in enumerate(("x", "y", "z", "rgb"))])
        publisher = node.create_publisher(PointCloud2, source_topic, qos)
        messages = [message]
    elif mode == "octomap":
        from octomap_msgs.msg import Octomap
        tree, resolution, binary, payload = octomap_file(record["path"]) if dataset else ("ColorOcTree", 0.2, False, octomap_bytes())
        message = Octomap(binary=binary, id=tree, resolution=resolution, data=array("b", payload))
        publisher = node.create_publisher(Octomap, source_topic, qos)
        messages = [message]
    elif mode == "nvblox":
        from nvblox_msgs.msg import Index3D, Mesh, MeshBlock
        publisher = node.create_publisher(Mesh, source_topic, qos)
        centroids = vertices[faces].mean(axis=1)
        keys = np.floor(centroids / 4).astype(int)
        unique = np.unique(keys, axis=0)
        message = Mesh(block_size_m=4.0)
        for key in unique:
            selected = np.all(keys == key, axis=1)
            xyz = vertices[faces[selected]].reshape(-1, 3)
            rgba = np.repeat(rgb[selected], 3, axis=0) / 255
            block = MeshBlock(vertices=[Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in xyz],
                triangles=list(range(len(xyz))),
                colors=[ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0) for c in rgba])
            message.block_indices.append(Index3D(x=int(key[0]), y=int(key[1]), z=int(key[2])))
            message.blocks.append(block)
        messages = [message]
    else:
        publisher = node.create_publisher(UInt8MultiArray if mode == "indexed" else Marker,
            source_topic + "/indexed" if mode == "indexed" else source_topic, qos)
        size = len(positions) if mode == "voxblox-marker" else 6000
        for start in range(0, len(positions), size):
            xyz = [tuple(map(float, p)) for p in positions[start:start + size]]
            rgba = [tuple(map(float, c)) for c in colors[start:start + size]]
            if mode == "indexed":
                message = UInt8MultiArray(data=array("B", build_indexed_binary_payload(start // size, xyz, rgba)))
            else:
                message = Marker(ns="mapping_demo", id=start // size, type=Marker.TRIANGLE_LIST,
                    action=Marker.ADD, points=[Point(x=x, y=y, z=z) for x, y, z in xyz],
                    colors=[ColorRGBA(r=r, g=g, b=b, a=a) for r, g, b, a in rgba])
                message.pose.orientation.w = 1.0
                message.scale.x = message.scale.y = message.scale.z = 1.0
                message.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            messages.append(message)

    cursor = 0
    next_cycle = 0.0

    def tick():
        nonlocal cursor, next_cycle, phase
        import time
        now = time.monotonic()
        if now < next_cycle:
            return
        message = messages[cursor]
        if hasattr(message, "header"):
            message.header.frame_id = "map"
            message.header.stamp = node.get_clock().now().to_msg()
        if mode == "points-refresh" and not dataset:
            # Visible, bounded translation of the generated source, not the workspace.
            packed["xyz"][:, 2] = xyz[:, 2] + 0.2 * np.sin(phase * 0.2)
            message.data = packed.tobytes()
        elif mode == "nvblox":
            message.clear = phase == 0
            # All source triangles remain; change only one block's colors every cycle.
            c = message.blocks[-1].colors[0]
            c.r = 1.0 if phase % 2 == 0 else 0.2
        publisher.publish(message)
        cursor += 1
        if cursor == len(messages):
            cursor = 0
            phase += 1
            next_cycle = now + (0.5 if mode == "points-refresh" else 30.0 if dataset else 5.0)

    node.create_timer(0.05, tick)
    summary = {"fixture": record["provenance"] if dataset else "synthetic industrial site" if mode != "octomap" else "synthetic colored voxel room",
               "path": mode, "topic": publisher.topic_name, "frame": "map",
               "triangles": len(faces) if mode not in ("octomap", "points", "points-refresh") else None,
               "points": len(xyz) if mode.startswith("points") else None,
               "messages_per_snapshot": len(messages)}
    print(json.dumps(summary), flush=True)
    if ready_file:
        ready_file.write_text(json.dumps(summary) + "\n")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
