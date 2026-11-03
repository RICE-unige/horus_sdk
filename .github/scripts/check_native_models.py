#!/usr/bin/env python3
"""Native robot compiler geometry/material contracts and actual DDS chunk delivery."""
import base64
import gzip
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType

FIXTURE = Path(__file__).resolve().parents[2] / "contracts/fixtures/robot_model/articulated.urdf"
LINK_COUNT = 1500


def resolve(binary, config):
    result = subprocess.run([binary, "--source", json.dumps(config)], check=True,
                            capture_output=True, text=True, timeout=30)
    return json.loads(result.stdout)


def source_and_texture_checks(binary, folder):
    from PIL import Image
    xml = FIXTURE.read_text().replace("triangle.obj", str(FIXTURE.with_name("triangle.obj")))
    node = rclpy.create_node("native_model_sources")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(String, "/native_model/description", qos)
    publisher.publish(String(data=xml))

    def parameters(request, response):
        assert request.names == ["robot_description"]
        response.values = [ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=xml)]
        return response

    node.create_service(GetParameters, "/native_model_source/get_parameters", parameters)
    try:
        for source in ("topic", "ros"):
            config = {"source": source, "robot_description_topic": "/native_model/description",
                      "ros_param_node": "/native_model_source", "source_timeout_s": 4}
            # File-backed output avoids blocking on a full pipe while spinning ROS.
            with tempfile.TemporaryFile(mode="w+") as output:
                process = subprocess.Popen([binary, "--source", json.dumps(config)],
                                           stdout=output, stderr=subprocess.PIPE, text=True)
                deadline = time.monotonic() + 8
                try:
                    while process.poll() is None and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.01)
                    assert process.poll() is not None, "Source resolver did not terminate"
                    _, stderr = process.communicate(timeout=1)
                    assert process.returncode == 0, stderr
                    output.seek(0)
                    assert json.load(output)["manifest"]["link_count"] == 3
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.communicate()
        # A missing explicit parameter must not select an unrelated topic robot.
        failure = subprocess.run([binary, "--source", json.dumps({
            "source": "ros", "ros_param_node": "/native_model_missing",
            "robot_description_topic": "/native_model/description", "source_timeout_s": 0.1})],
            capture_output=True, text=True, timeout=5)
        assert failure.returncode != 0 and "Timed out" in failure.stderr, failure
    finally:
        node.destroy_node()

    xacro = folder / "source.urdf.xacro"
    expression = "$" + "{size}"
    xacro.write_text('<robot name="xacro" xmlns:xacro="http://www.ros.org/wiki/xacro">'
                     '<xacro:property name="size" value="0.25"/><link name="base">'
                     f'<collision><geometry><sphere radius="{expression}"/></geometry></collision>'
                     '</link></robot>')
    assert resolve(binary, {"urdf_path": str(xacro)})["payload"]["links"][0]["collisions"][0]["radius"] == 0.25

    obj = folder / "textured.obj"
    obj.write_text("mtllib textured.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\n"
                   "vt 0 0\nvt 1 0\nvt 0 1\nusemtl texture\nf 1/1 2/2 3/3\n")
    urdf = folder / "textured.urdf"
    urdf.write_text(f'<robot name="texture"><link name="base"><visual><geometry>'
                    f'<mesh filename="{obj}"/></geometry></visual></link></robot>')
    for extension in ("png", "jpg"):
        texture = folder / ("color." + extension)
        Image.new("RGB", (4, 4), (128, 64, 192)).save(texture)
        (folder / "textured.mtl").write_text(f"newmtl texture\nKd 1 1 1\nmap_Kd {texture.name}\n")
        artifact = resolve(binary, {"urdf_path": str(urdf), "preserve_source_mesh": True})
        colors = base64.b64decode(artifact["payload"]["mesh_assets"][0]["colors_b64"])
        with Image.open(texture) as decoded:
            rgb = decoded.convert("RGB")
            pixels = [rgb.getpixel((x, y)) for y in range(rgb.height) for x in range(rgb.width)]
        linear = lambda c: c/12.92 if c <= 0.04045 else ((c+0.055)/1.055)**2.4
        expected = tuple(round(sum(linear(p[k]/255) for p in pixels)/len(pixels)*255) for k in range(3))
        assert all(abs(colors[k]-expected[k]) <= 1 for k in range(3)), (extension, colors[:4], expected)
        texture.write_bytes(b"not an image")
        failure = subprocess.run([binary, "--source", json.dumps({"urdf_path": str(urdf)})],
                                 capture_output=True, text=True, timeout=10)
        assert failure.returncode != 0, "Broken material silently produced a white mesh"
    print(f"PASS {binary}: retained/topic and parameter sources, source isolation, xacro, PNG/JPEG linear colors", flush=True)


def compiler(binary):
    result = subprocess.run([binary, str(FIXTURE), "--artifact"], check=True, capture_output=True, text=True, timeout=30)
    artifact = json.loads(result.stdout)
    raw = artifact["payload_json"].encode()
    assert gzip.decompress(base64.b64decode(artifact["encoded_payload"])) == raw
    assert artifact["manifest"]["description_id"] == "sha256:" + hashlib.sha256(raw).hexdigest()
    payload = artifact["payload"]
    assert payload["visual_link_pose_source"] == "tf"
    assets = {a["mesh_id"]: a for a in payload["mesh_assets"]}
    for visual in payload["visual_links"]:
        asset = assets[visual["mesh_id"]]
        colors = base64.b64decode(asset["colors_b64"])
        assert len(colors) == asset["vertex_count"] * 4
        palette = set(zip(*[iter(colors)] * 4))
        assert (204, 26, 51, 255) in palette, palette
        if visual["name"] == "base":
            assert (26, 51, 204, 255) in palette, palette
        indices = base64.b64decode(asset["indices_b64"])
        assert max(struct.unpack("<" + "I" * (len(indices)//4), indices)) < asset["vertex_count"]
    # Compare the geometric contract rather than vertex ordering or compiler IDs.
    from horus.description.robot_description_resolver import RobotDescriptionResolver
    resolver = RobotDescriptionResolver()
    config = resolver._parse_config({"robot_description_config": {
        "urdf_path": str(FIXTURE), "base_frame": "base", "source": "local",
        "body_mesh_mode": "runtime_high_mesh", "visual_link_pose_source": "tf"}})
    class Robot:
        name = "articulated"
        dimensions = None
    reference = resolver._compile_robot_description_payload(Robot(), FIXTURE.read_text(), config).to_dict()
    assert [v["frame_id"] for v in payload["visual_links"]] == [v["frame_id"] for v in reference["visual_links"]]
    for actual, expected in zip(payload["visual_links"], reference["visual_links"]):
        for key in ("origin_xyz", "origin_rpy"):
            assert all(abs(a-b) < 1e-5 for a,b in zip(actual[key], expected[key])), (actual, expected)
    assert len(payload["joints"]) == len(reference["joints"])
    for actual, expected in zip(payload["joints"], reference["joints"]):
        for key in ("origin_xyz", "origin_rpy", "axis_xyz"):
            assert all(abs(a-b) < 1e-5 for a,b in zip(actual[key], expected[key]))
    print(f"PASS {binary}: model compiler, Python poses, colors and gzip/hash contract", flush=True)


def delivery(binary, path):
    node = rclpy.create_node("native_model_peer")
    ack = node.create_publisher(String, "/horus/registration_ack", 32)
    request = node.create_publisher(String, "/horus/robot_description/request", 32)
    registration = []
    begins, items, ends = {}, {}, {}
    retained = QoSProfile(depth=32, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

    def registered(message):
        payload = json.loads(message.data)
        if payload.get("robot_name") != "native_model":
            return
        registration.append(payload)
        ack.publish(String(data=json.dumps({"success": True, "robot_name": "native_model", "robot_id": "model-id"})))

    node.create_subscription(String, "/horus/registration", registered, retained)
    node.create_subscription(String, "/native_model/reply/begin", lambda msg: begins.update({json.loads(msg.data)["request_id"]: json.loads(msg.data)}), 32)
    def item(message):
        value = json.loads(message.data)
        items.setdefault(value["request_id"], {})[value["chunk_index"]] = value["chunk_data"]
    node.create_subscription(String, "/native_model/reply/item", item, 32)
    node.create_subscription(String, "/native_model/reply/end", lambda msg: ends.update({json.loads(msg.data)["request_id"]: json.loads(msg.data)}), 32)
    process = subprocess.Popen([binary, str(path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def until(condition, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError(process.communicate())
            rclpy.spin_once(node, timeout_sec=0.01)
            if condition():
                return
        raise AssertionError(f"Model DDS timeout: registrations={len(registration)}, begins={begins}, ends={ends}")

    try:
        until(lambda: registration)
        assert "robot_description_payload_json" not in registration[-1], "Large model leaked into registration channel"
        manifest = registration[-1]["robot_description_manifest"]
        for sequence in range(2):
            request_id = f"model-repair-{sequence}"
            request.publish(String(data=json.dumps({"request_id": request_id, "robot_name": "native_model", "description_id": manifest["description_id"], "session_id": "test", "app_id": "app", "chunk_begin_topic": "/native_model/reply/begin", "chunk_item_topic": "/native_model/reply/item", "chunk_end_topic": "/native_model/reply/end"})))
            until(lambda: request_id in begins and request_id in ends and len(items.get(request_id, {})) == begins[request_id]["expected_chunks"])
            chunks = items[request_id]
            raw = gzip.decompress(base64.b64decode("".join(chunks[i] for i in range(len(chunks)))))
            assert "sha256:" + hashlib.sha256(raw).hexdigest() == manifest["description_id"]
            payload = json.loads(raw)
            assert payload["robot_name"] == "native_model", "URDF name replaced registration identity"
            assert len(payload["visual_links"]) == LINK_COUNT
            assert len(payload["mesh_assets"]) == 1, "Repeated identical geometry was not shared"
            assert begins[request_id]["session_id"] == "test" and ends[request_id]["received_count"] == len(chunks)
        while process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
        stdout, stderr = process.communicate(timeout=1)
        assert process.returncode == 0, (stdout, stderr)
        print(f"PASS {binary}: {LINK_COUNT} articulated links, shared geometry, paced delivery and repeat recovery", flush=True)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
        node.destroy_node()


if __name__ == "__main__":
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        raise SystemExit("Use an isolated localhost ROS domain")
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix="horus-native-model-") as folder:
            path = Path(folder)/"many.urdf"
            mesh = FIXTURE.with_name("triangle.obj")
            links = [f'<link name="{name}"><visual><geometry><mesh filename="{mesh}"/></geometry></visual></link>' for name in ["base"] + [f"part{i}" for i in range(LINK_COUNT-1)]]
            joints = [f'<joint name="joint{i}" type="revolute"><parent link="base"/><child link="part{i}"/><origin xyz="{i*.01} 0 0"/></joint>' for i in range(LINK_COUNT-1)]
            path.write_text('<robot name="many">'+"".join(links+joints)+'</robot>')
            for binary in sys.argv[1:]:
                compiler(binary)
                source_and_texture_checks(binary, Path(folder))
                delivery(binary, path)
    finally:
        rclpy.shutdown()
