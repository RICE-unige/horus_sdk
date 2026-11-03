#!/usr/bin/env python3
"""Exercise complete native CUDA scene-to-ROS launchers, with no headset or robot."""
import argparse
import copy
import io
import json
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np
from PIL import Image as PillowImage
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from horus.remote_rendering.frame_protocol import decode_frame_marker
from horus.remote_rendering.v3_protocol import unpack_ros_diagnostic_frame


def check(binary, library, scene, mode, index, video_library="", gaussian=False):
    root = Path(__file__).resolve().parents[2]
    node = rclpy.create_node(f"native_agent_peer_{index}")
    frames, statuses = [], []
    prefix = f"/native_agent_{index}"
    qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image if mode == "rgb" else CompressedImage,
                             prefix + "/image", frames.append,
                             qos if mode == "diagnostic" else QoSProfile(
                                 depth=64, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(String, prefix + "/status",
                             lambda msg: statuses.append(json.loads(msg.data)), 32)
    publisher = node.create_publisher(String, prefix + "/view", qos)
    template = json.loads((root / "contracts/fixtures/remote_protocol.json").read_text())["view"]
    for name in ("left_eye", "right_eye"):
        template[name]["position"] = dict(x=-0.03 if name == "left_eye" else 0.03, y=0, z=0)
        template[name]["rotation"] = dict(x=0, y=0, z=0, w=1)
        template[name]["fov"] = dict(left=-1, right=1, up=1, down=-1)
    with tempfile.TemporaryDirectory(prefix="horus-native-agent-") as directory:
        config_path = Path(directory) / "config.json"
        config_path.write_text(json.dumps(dict(
            mode=mode, view_topic=prefix + "/view", stream_topic=prefix + "/image",
            status_topic=prefix + "/status", maximum_view_age_seconds=0.4,
            width=192, height=48, fps=30, depth_strip_height=16,
            duration_seconds=9, near_m=0.1, far_m=10, bake_lighting=False,
            video_library=video_library, scene_kind="gaussian" if gaussian else "geometry")))
        with tempfile.TemporaryFile(mode="w+") as stdout, tempfile.TemporaryFile(mode="w+") as stderr:
            process = subprocess.Popen([binary, str(scene), library, str(config_path)],
                                       stdout=stdout, stderr=stderr, text=True)
            start = time.monotonic()
            connected_at, last_send, sequence = None, 0, 0
            try:
                while process.poll() is None and time.monotonic() - start < 14:
                    now = time.monotonic()
                    if publisher.get_subscription_count() and connected_at is None:
                        connected_at = now
                    elapsed = 0 if connected_at is None else now - connected_at
                    if connected_at is not None and now - last_send > 0.04 and (
                            elapsed < 2 or 3.2 < elapsed < 5.2):
                        sequence += 1
                        view = copy.deepcopy(template)
                        view["view_sequence"] = sequence
                        view["sample_time_ns"] = sequence * 1_000_000
                        if elapsed > 3.2:
                            view["view_stream_id"] = "reconnected-viewer"
                            view["workspace"]["revision"] = 5
                        publisher.publish(String(data=json.dumps(view)))
                        if sequence % 7 == 0:
                            publisher.publish(String(data=json.dumps(view)))
                            publisher.publish(String(data="{invalid"))
                        last_send = now
                    rclpy.spin_once(node, timeout_sec=0.005)
                    if connected_at is not None and elapsed > 6.0:
                        process.terminate()  # Exercise graceful SIGTERM, not only fixed duration.
                        process.wait(timeout=3)
                process.wait(timeout=3)
                stdout.seek(0)
                stderr.seek(0)
                output, errors = stdout.read(), stderr.read()
                assert process.returncode == 0, output + errors
                final = json.loads(output.strip().splitlines()[-1])
                assert final["state"] == "stopped" and not final["running"], final
                assert not final["error"] and final["views_rejected"] > 0, final
                assert final["scene_vertices"] == (0 if gaussian else 3)
                assert final["scene_gaussians"] == (3 if gaussian else 0)
                assert final["device_video"] == bool(video_library)
                assert final["scene_triangles"] == (1 if scene.suffix == ".obj" else 0)
                assert any(s["state"] == "stale_view" for s in statuses), statuses
                assert len(frames) >= 12, final
                metadata_list, images = [], []
                for frame in frames:
                    if mode == "diagnostic":
                        metadata, png = unpack_ros_diagnostic_frame(bytes(frame.data))
                        metadata = metadata.to_dict()
                        images.append(np.asarray(PillowImage.open(io.BytesIO(png)).convert("RGB")))
                    else:
                        metadata = json.loads(frame.header.frame_id.split(":", 1)[1])
                        if mode == "rgb":
                            assert frame.encoding == "rgb8" and frame.step == 384 * 3
                            images.append(np.frombuffer(bytes(frame.data), np.uint8).reshape(80, 384, 3))
                    assert int(start * 1e9) <= metadata["render_start_ns"] <= metadata["render_complete_ns"]
                    assert metadata["render_complete_ns"] <= metadata["capture_time_ns"] <= time.monotonic_ns()
                    assert 0 < metadata["source_view_sequence"] <= sequence
                    metadata_list.append(metadata)
                if mode == "h264":
                    video = Path(directory) / "frames.h264"
                    video.write_bytes(b"".join(bytes(f.data) for f in frames))
                    decoded = subprocess.run(["ffmpeg", "-v", "error", "-i", str(video),
                                              "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
                                             capture_output=True, check=True, timeout=20)
                    assert not decoded.stderr, decoded.stderr.decode()
                    images = np.frombuffer(decoded.stdout, np.uint8).reshape(-1, 80, 384, 3)
                    assert len(images) == len(frames)
                    probe = subprocess.run(["ffprobe", "-v", "error", "-show_frames", "-of",
                                            "json", str(video)], capture_output=True, check=True, timeout=20)
                    keyframes = json.loads(probe.stdout)["frames"]
                    for i, metadata in enumerate(metadata_list):
                        if i == 0 or metadata["workspace_revision"] != metadata_list[i - 1]["workspace_revision"]:
                            assert keyframes[i]["key_frame"], "New view identity must begin with IDR"
                assert {m["workspace_revision"] for m in metadata_list} == {4, 5}
                assert len({m["frame_id"] for m in metadata_list}) == len(metadata_list)
                for metadata, image in zip(metadata_list, images):
                    assert image.shape == (48 if mode == "diagnostic" else 80, 384, 3)
                    assert np.count_nonzero(image[:48]) >= 6, "Blank scene output"
                    if mode != "diagnostic":
                        assert decode_frame_marker(image[-16:]) == metadata["frame_id"] % 65536
                print(json.dumps(dict(binary=binary, scene=scene.name, mode=mode, result="passed",
                                      frames=len(frames), full_geometry=True, clock="host_monotonic",
                                      stale_detection=True, viewer_restart=True, graceful_sigterm=True)), flush=True)
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--point-library", required=True)
    parser.add_argument("--mesh-library", required=True)
    parser.add_argument("--nvenc", action="store_true")
    parser.add_argument("--video-library", default="")
    parser.add_argument("--gaussian-library", default="")
    parser.add_argument("binaries", nargs="+")
    args = parser.parse_args()
    fixtures = Path(__file__).resolve().parents[2] / "contracts/fixtures/remote_scene"
    rclpy.init()
    try:
        index = 0
        for binary in args.binaries:
            for scene, library in (("points.ply", args.point_library), ("triangle.obj", args.mesh_library)):
                for mode in ["diagnostic", "rgb"] + (["h264"] if args.nvenc else []):
                    check(binary, library, fixtures / scene, mode, index,
                          args.video_library if mode == "h264" else "")
                    index += 1
            if args.gaussian_library:
                with tempfile.TemporaryDirectory(prefix="horus-gaussian-agent-") as directory:
                    scene = Path(directory)/"gaussian.ply"
                    names = ["x","y","z","scale_0","scale_1","scale_2","rot_0","rot_1","rot_2","rot_3","opacity","f_dc_0","f_dc_1","f_dc_2"]
                    scene.write_text("ply\nformat ascii 1.0\nelement vertex 3\n"+
                        "".join(f"property float {n}\n" for n in names)+"end_header\n"+
                        "\n".join(f"{x} 0 2 -1 -1 -1 1 0 0 0 4 1 0 0" for x in [-.5,0,.5])+"\n")
                    for mode in ["diagnostic","rgb"]+(["h264"] if args.nvenc else []):
                        check(binary,args.gaussian_library,scene,mode,index,
                              args.video_library if mode == "h264" else "",gaussian=True)
                        index += 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
