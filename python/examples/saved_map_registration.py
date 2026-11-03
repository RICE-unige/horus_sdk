#!/usr/bin/env python3
"""Replay a verified, full-detail HMSH map fixture in an isolated MR workspace."""
import argparse
from dataclasses import dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import threading
import time

import numpy as np

from horus.mapping.mesh_page_scheduler import MeshPageScheduler
from horus.mapping.mesh_protocol import MeshPageUpdate, decode_live_header


@dataclass(frozen=True)
class SavedMap:
    metadata: dict
    pages: tuple
    manifest_sha256: str

    def replay_pages(self, epoch):
        if not 0 < epoch < 2**64:
            raise ValueError("Replay epoch must fit uint64")
        return tuple(replace(page, epoch=epoch,
            payload=page.payload[:24] + struct.pack("<Q", epoch) + page.payload[32:]) for page in self.pages)


def load_saved_map(directory):
    directory = Path(directory).expanduser().resolve()
    manifest = (directory / "manifest.json").read_bytes()
    metadata = json.loads(manifest)
    if metadata.get("schema") != "horus.mesh_fixture.v1" or not 1 <= len(metadata.get("pages", [])) <= 1024:
        raise ValueError("Expected a complete HORUS mesh fixture with 1..1024 pages")
    if metadata.get("input_inventory_verified") is False:
        raise ValueError("Saved source inventory is incomplete or unverified; inspect its capture metadata before replay")
    for field in ("source_floor_z_m", "workspace_scale"):
        if type(metadata.get(field)) not in (int, float) or not math.isfinite(metadata[field]):
            raise ValueError(f"Fixture needs a finite {field}")
    if not 0.001 <= metadata["workspace_scale"] <= 10:
        raise ValueError("Invalid fixture workspace scale")
    pages = []
    ids = set()
    expected = {p["chunk_id"]: int(p["revision"]) for p in metadata["original_manifest"]["pages"] if p["operation"] == 0}
    for entry in metadata["pages"]:
        path = (directory / entry["file"]).resolve()
        if not path.is_relative_to(directory):
            raise ValueError("Page must belong to the fixture directory")
        payload = path.read_bytes()
        if hashlib.sha256(payload).hexdigest() != entry["sha256"]:
            raise ValueError(f"Page checksum mismatch: {path.name}")
        header = decode_live_header(payload)
        if (header.operation != 0 or payload[41] not in (0, 1) or
            not 0 <= header.chunk_id <= 0x3FFFFFFF or header.chunk_id in ids or
            str(header.epoch) != metadata["original_manifest"]["epoch"] or
            header.chunk_id != entry["chunk_id"] or str(header.revision) != entry["revision"] or
            header.chunk_id not in expected or header.revision < expected[header.chunk_id]):
            raise ValueError("Invalid, duplicated or mismatched map page identity")
        stride = 16 if header.has_color else 12
        required = 52 + header.vertex_count * stride + header.index_count * 4
        if (not header.vertex_count or not header.index_count or header.index_count % 3 or
            required != len(payload) or len(payload) > 64 * 1024 * 1024):
            raise ValueError("Incomplete mesh page geometry")
        positions = np.ndarray((header.vertex_count, 3), dtype="<f4", buffer=payload,
            offset=52, strides=(stride, 4))
        indices = np.frombuffer(payload, dtype="<u4", count=header.index_count,
            offset=52 + header.vertex_count * stride)
        if not np.isfinite(positions).all() or indices.max() >= header.vertex_count:
            raise ValueError("Invalid mesh coordinates or indices")
        if header.vertex_count != entry["vertex_count"] or header.index_count // 3 != entry["triangle_count"]:
            raise ValueError("Page counts do not match the fixture manifest")
        ids.add(header.chunk_id)
        pages.append(MeshPageUpdate(header.chunk_id, header.page_key, header.epoch, header.revision,
            header.operation, payload, header.vertex_count, header.index_count // 3))
    if (ids != set(expected) or sum(p.triangle_count for p in pages) != metadata["triangles"] or
        sum(p.vertex_count for p in pages) != metadata["vertices"] or
        sum(len(p.payload) for p in pages) != metadata["payload_bytes"]):
        raise ValueError("Fixture totals do not match its complete page inventory")
    return SavedMap(metadata, tuple(pages), hashlib.sha256(manifest).hexdigest())


def create_source(fixture, topic="/horus/saved_map/mesh"):
    """Use the normal live page/repair path, with immutable real geometry."""
    from array import array
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String, UInt8MultiArray
    from tf2_ros import StaticTransformBroadcaster

    class Source(Node):
        def __init__(self):
            super().__init__("horus_saved_map")
            epoch = max(time.time_ns(), fixture.pages[0].epoch + 1)
            self.pages = fixture.replay_pages(epoch)
            self.by_id = {p.chunk_id: p for p in self.pages}
            self.scheduler = MeshPageScheduler(pages_per_second=30, bytes_per_second=4 * 1024 * 1024,
                max_pages=1024, max_payload_bytes=max(len(p.payload) for p in self.pages), now=time.monotonic())
            qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE)
            self.publisher = self.create_publisher(UInt8MultiArray, topic + "/indexed", qos)
            self.inventory = self.create_publisher(String, topic + "/indexed_manifest",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.request = self.create_subscription(String, topic + "/indexed_request", self.repair, 8)
            self.manifest = dict(version=1, epoch=str(epoch), frame_id="saved_map/source",
                pages=[dict(chunk_id=p.chunk_id, revision=str(p.revision), operation=0) for p in self.pages],
                triangles=fixture.metadata["triangles"])
            self.tf = StaticTransformBroadcaster(self)
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = "saved_map/source"
            transform.transform.translation.z = -fixture.metadata["source_floor_z_m"]
            transform.transform.rotation.w = 1.0
            self.tf.sendTransform(transform)
            self.had_subscriber = False
            self.create_timer(.01, self.publish_page)
            self.create_timer(2.0, self.publish_manifest)
            self.publish_manifest()

        def publish_manifest(self):
            self.inventory.publish(String(data=json.dumps(self.manifest, separators=(",", ":"))))

        def repair(self, message):
            try:
                request = json.loads(message.data)
            except (ValueError, TypeError):
                return
            if not isinstance(request, dict) or request.get("request") not in ("snapshot", "pages"):
                return
            if request["request"] == "snapshot" or str(request.get("epoch")) != self.manifest["epoch"]:
                selected = self.pages
            else:
                ids = request.get("ids")
                if not isinstance(ids, list) or len(ids) > 1024 or any(type(i) is not int for i in ids):
                    return
                selected = [self.by_id[i] for i in set(ids) if i in self.by_id]
            for page in selected:
                self.scheduler.enqueue(page)

        def publish_page(self):
            connected = self.publisher.get_subscription_count() > 0
            if connected and not self.had_subscriber:
                for page in self.pages:
                    self.scheduler.enqueue(page)
            self.had_subscriber = connected
            if not connected:
                return
            page = self.scheduler.pop_ready(time.monotonic())
            if page is not None:
                self.publisher.publish(UInt8MultiArray(data=array("B", page.payload)))

    return Source()


def remote_arrays(fixture):
    """Keep full source topology/RGB, applying only the local replay's floor TF."""
    vertices = np.empty((fixture.metadata["vertices"], 3), dtype=np.float32)
    colors = np.full(vertices.shape, 255, dtype=np.uint8)
    faces = np.empty((fixture.metadata["triangles"], 3), dtype=np.int32)
    if len(vertices) > np.iinfo(np.int32).max:
        raise ValueError("Saved map exceeds the remote renderer index range")
    vo = fo = 0
    for page in fixture.pages:
        header = decode_live_header(page.payload)
        stride = 16 if header.has_color else 12
        xyz = np.ndarray((header.vertex_count, 3), dtype="<f4", buffer=page.payload,
            offset=52, strides=(stride, 4))
        vertices[vo:vo + header.vertex_count] = xyz
        if header.has_color:
            rgba = np.ndarray((header.vertex_count, 4), dtype=np.uint8,
                buffer=page.payload, offset=64, strides=(stride, 1))
            if np.any(rgba[:, 3] != 255):
                raise ValueError("Remote saved-map replay requires opaque colours; alpha cannot be discarded")
            colors[vo:vo + header.vertex_count] = rgba[:, :3]
        indices = np.frombuffer(page.payload, dtype="<u4", count=header.index_count,
            offset=52 + header.vertex_count * stride).reshape(-1, 3)
        faces[fo:fo + page.triangle_count] = indices + vo
        vo += header.vertex_count
        fo += page.triangle_count
    vertices[:, 2] -= fixture.metadata["source_floor_z_m"]
    if not np.isfinite(vertices).all():
        raise ValueError("Saved-map floor alignment produced non-finite coordinates")
    return dict(vertices=vertices, colors=colors, faces=faces)


def prepare_remote_map(fixture, cache_root):
    """Verify cached arrays before reuse; never write into the source fixture."""
    arrays = remote_arrays(fixture)
    root = Path(cache_root).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    path = root / (fixture.manifest_sha256 + "-remote-v1.npz")
    valid = False
    if path.is_file():
        try:
            with np.load(path, allow_pickle=False) as data:
                valid = all(key in data and data[key].dtype == value.dtype and
                    np.array_equal(data[key], value) for key, value in arrays.items())
        except (OSError, ValueError, EOFError):
            valid = False
    if not valid:
        with tempfile.NamedTemporaryFile(dir=root, suffix=".npz", delete=False) as stream:
            temporary = Path(stream.name)
        try:
            np.savez_compressed(temporary, **arrays)
            with np.load(temporary, allow_pickle=False) as data:
                if not all(np.array_equal(data[key], value) for key, value in arrays.items()):
                    raise RuntimeError("Remote fixture export failed round-trip verification")
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
    return path


def native_render_options(profile):
    if profile not in ("off", "lod", "spacewarp", "combined"):
        raise ValueError("Unknown native mesh render profile")
    from horus.dataviz.mesh_display import mesh_display_options
    return {"display": mesh_display_options({
        "lod_enabled": profile in ("lod", "combined"),
        "spacewarp_enabled": profile in ("spacewarp", "combined"),
    })}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", type=Path)
    parser.add_argument("--check", action="store_true", help="Verify the complete fixture without starting ROS or the bridge")
    parser.add_argument("--render-target", choices=("quest", "remote"), default="quest")
    parser.add_argument("--mesh-render-profile", choices=("off", "lod", "spacewarp", "combined"), default="off",
        help="Native Quest presentation; LOD approximates distant geometry, SpaceWarp synthesizes display frames")
    parser.add_argument("--profile", choices=("fast", "balanced", "quality"), default="quality",
        help="Remote stereo-video profile; geometry always remains full detail")
    parser.add_argument("--remote-cache", type=Path,
        default=Path.home() / ".cache/horus/saved-maps",
        help="Derived remote RGB arrays; original fixture is never modified")
    args = parser.parse_args(argv)
    fixture = load_saved_map(args.fixture)
    print(f"Verified {len(fixture.pages)} pages / {fixture.metadata['triangles']:,} triangles; no geometry changes.", flush=True)
    if args.check:
        return 0
    if args.render_target == "remote":
        from remote_map_portal_registration import main as remote_main
        path = prepare_remote_map(fixture, args.remote_cache)
        print(f"Remote full-map source: {path}; same floor alignment; no simplification.", flush=True)
        return remote_main(["--scene", "recorded_mesh", "--dataset-path", str(path),
            "--profile", args.profile, "--workspace-scale", str(fixture.metadata["workspace_scale"]),
            "--protocol", "v3", "--transport", "webrtc"])
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from horus.robot import WorkspaceVisualization, register_entities, is_registration_cancelled
    rclpy.init()
    source = create_source(fixture)
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        resource = WorkspaceVisualization("saved_map")
        viz = resource.create_dataviz("saved_map")
        options = native_render_options(args.mesh_render_profile)
        print(f"Native mesh render profile: {args.mesh_render_profile}; source geometry remains unchanged.", flush=True)
        viz.add_nvblox_mesh("/horus/saved_map/mesh", frame_id="map", render_options=options)
        ok, result = register_entities([resource], datavizs=[viz],
            workspace_scale=fixture.metadata["workspace_scale"], compass_enabled=False, keep_alive=True)
        return 0 if ok or is_registration_cancelled(result) else 1
    finally:
        executor.shutdown(timeout_sec=2)
        worker.join(timeout=2)
        source.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
