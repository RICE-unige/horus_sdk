#!/usr/bin/env python3
"""Stream ``nvblox_msgs/msg/Mesh`` to HORUS as full-geometry live mesh pages.

The adapter keeps nvblox's incremental block state on the PC, merges neighboring
blocks into renderer-sized spatial pages, and publishes HMSH v2 page replacements.
No triangles are decimated. Coordinates use float32 and colors use RGBA8.
The Quest receives changed pages and requested repair/replay pages.
"""

from __future__ import annotations

import argparse
from array import array
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time

from horus.mapping.nvblox_mesh import (
    LiveNvbloxMeshState,
    MeshPageUpdate,
    PendingNvbloxMesh,
)
from horus.mapping.mesh_page_scheduler import MeshPageScheduler


def parse_color(value: str) -> tuple[float, float, float, float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) not in (3, 4):
        raise argparse.ArgumentTypeError("color must be r,g,b or r,g,b,a")
    try:
        channels = [float(part) for part in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("color channels must be numbers") from exc
    if any(channel > 1.0 for channel in channels):
        channels = [channel / 255.0 for channel in channels]
    if len(channels) == 3:
        channels.append(1.0)
    if not all(math.isfinite(channel) for channel in channels):
        raise argparse.ArgumentTypeError("color channels must be finite")
    return tuple(max(0.0, min(1.0, channel)) for channel in channels)  # type: ignore[return-value]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/nvblox_node/mesh")
    parser.add_argument("--output-topic", default="/horus/nvblox/mesh")
    parser.add_argument("--presentation-frame", default="",
                        help="Optional coordinate-identical frame alias for a separately published visual datum TF. Does not transform vertices.")
    parser.add_argument("--input-reliability", choices=("auto", "reliable", "best_effort"), default="auto",
                        help="Auto waits for source discovery and requests reliable delivery when all publishers offer it.")
    parser.add_argument(
        "--max-publish-mib-per-second", type=float, default=8.0,
        help="Byte-rate ceiling for full-detail map updates; controls delivery time, not geometry.",
    )
    parser.add_argument(
        "--blocks-per-axis",
        type=int,
        default=4,
        help="Merge NxNxN neighboring nvblox blocks per Quest renderer without decimation.",
    )
    parser.add_argument("--max-pages", type=int, default=1024)
    parser.add_argument("--max-payload-mib", type=float, default=16.0)
    parser.add_argument(
        "--color-mode",
        choices=("auto", "source", "normals", "fallback"),
        default="auto",
        help="Use source colors, or normal-axis colors when the mesh is uncolored.",
    )
    parser.add_argument("--fallback-color", type=parse_color, default=parse_color("178,184,198,255"))
    parser.add_argument(
        "--max-publish-pages-per-second",
        type=float,
        default=90.0,
        help="Bound ROS/bridge traffic while preserving every latest page revision.",
    )
    parser.add_argument(
        "--snapshot-replay-seconds",
        type=float,
        default=0.0,
        help="Optional repair replay interval. Zero uses request replay; output QoS is volatile.",
    )
    args = parser.parse_args()
    if not args.input_topic.startswith("/") or not args.output_topic.startswith("/"):
        parser.error("input and output topics must be absolute")
    if not 1 <= args.blocks_per_axis <= 8:
        parser.error("--blocks-per-axis must be between 1 and 8")
    if not 1 <= args.max_pages <= 1024:
        parser.error("--max-pages must be between 1 and 1024")
    if not 0.25 <= args.max_payload_mib <= 64.0:
        parser.error("--max-payload-mib must be between 0.25 and 64")
    if not 1.0 <= args.max_publish_pages_per_second <= 500.0:
        parser.error("--max-publish-pages-per-second must be between 1 and 500")
    if not math.isfinite(args.max_publish_mib_per_second) or args.max_publish_mib_per_second <= 0:
        parser.error("--max-publish-mib-per-second must be finite and > 0")
    if not math.isfinite(args.snapshot_replay_seconds) or args.snapshot_replay_seconds < 0:
        parser.error("--snapshot-replay-seconds must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    try:
        import rclpy
        from nvblox_msgs.msg import Mesh
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String, UInt8MultiArray
    except ImportError as exc:
        raise SystemExit(
            f"Missing ROS 2 Python dependency ({exc}). Source ROS first; if "
            "nvblox_msgs is absent, install it with: sudo apt install "
            "ros-$ROS_DISTRO-nvblox-msgs"
        ) from exc

    class NvbloxMeshAdapter(Node):
        def __init__(self) -> None:
            super().__init__("horus_nvblox_mesh_adapter")
            self.state = LiveNvbloxMeshState(
                blocks_per_axis=args.blocks_per_axis,
                max_pages=args.max_pages,
                max_payload_bytes=int(args.max_payload_mib * 1024 * 1024),
                color_mode=args.color_mode,
                fallback_color=args.fallback_color,
            )
            self.scheduler = MeshPageScheduler(
                pages_per_second=args.max_publish_pages_per_second,
                bytes_per_second=args.max_publish_mib_per_second * 1024 * 1024,
                max_pages=args.max_pages,
                max_payload_bytes=self.state.max_payload_bytes,
                now=time.monotonic(),
            )
            self.received_messages = 0
            self.published_pages = 0
            self.last_input_time = 0.0
            self.last_log_time = 0.0
            self.published_bytes = 0
            self.last_snapshot_time = -math.inf
            self.pending_source = PendingNvbloxMesh(max_blocks=args.max_pages * args.blocks_per_axis ** 3)
            self.mesh_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="horus-mesh")
            self.mesh_future = None
            self.committed_manifest = self.state.manifest()
            self.committed_snapshot = []
            self.committed_blocks = 0
            self.last_encode_seconds = 0.0

            output_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=64,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            request_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=16,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.publisher = self.create_publisher(
                UInt8MultiArray,
                f"{args.output_topic}/indexed",
                output_qos,
            )
            self.manifest_publisher = self.create_publisher(
                String, f"{args.output_topic}/indexed_manifest",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.subscription = None
            self.source_delivery = "waiting_for_discovery"
            self.connect_source()
            self.discovery_timer = self.create_timer(0.5, self.connect_source)
            self.request_subscription = self.create_subscription(
                String,
                f"{args.output_topic}/indexed_request",
                self.on_snapshot_request,
                request_qos,
            )
            self.publish_timer = self.create_timer(0.01, self.publish_pending)
            self.status_timer = self.create_timer(2.0, self.log_status)
            self.replay_timer = (
                self.create_timer(args.snapshot_replay_seconds, self.enqueue_snapshot)
                if args.snapshot_replay_seconds > 0
                else None
            )
            self.get_logger().info(
                f"nvblox live mesh: {args.input_topic} -> {args.output_topic}/indexed; "
                f"page={args.blocks_per_axis}^3 blocks, rate<={args.max_publish_pages_per_second:.0f}/s, "
                f"bytes<={args.max_publish_mib_per_second:g} MiB/s"
            )

        def connect_source(self) -> None:
            if self.subscription is not None:
                return
            endpoints = self.get_publishers_info_by_topic(args.input_topic)
            if args.input_reliability == "auto" and not endpoints:
                return
            reliable = args.input_reliability == "reliable" or (
                args.input_reliability == "auto" and
                all(info.qos_profile.reliability == ReliabilityPolicy.RELIABLE for info in endpoints)
            )
            transient = bool(endpoints) and all(
                info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for info in endpoints
            )
            input_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=8,
                reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE,
            )
            self.subscription = self.create_subscription(Mesh, args.input_topic, self.on_mesh, input_qos)
            self.source_delivery = "reliable" if reliable else "best_effort"
            self.get_logger().info(f"Source delivery: {self.source_delivery}; transient_local={transient}")
            if not reliable:
                self.get_logger().warning(
                    "Best-effort incremental mesh input can lose blocks before the HORUS cache. "
                    "Use a reliable source and start this adapter before mapping, or request a full source snapshot."
                )

        def on_mesh(self, message: Mesh) -> None:
            try:
                self.pending_source.add(message)
            except (ValueError, RuntimeError) as exc:
                self.get_logger().error(f"Rejected nvblox mesh update: {exc}")
                return
            self.received_messages += 1
            self.last_input_time = time.monotonic()

        def encode_batch(self, batch):
            started = time.monotonic()
            updates = self.state.apply_message(batch)
            return (updates, self.state.manifest(), self.state.snapshot_updates(include_deletes=True),
                    self.state.block_count, time.monotonic() - started)

        def advance_source(self):
            if self.mesh_future is not None:
                if not self.mesh_future.done():
                    return
                try:
                    updates, manifest, snapshot, blocks, seconds = self.mesh_future.result()
                    self.committed_manifest, self.committed_snapshot = manifest, snapshot
                    self.committed_blocks, self.last_encode_seconds = blocks, seconds
                    for update in updates:
                        self.enqueue(update)
                except Exception as exc:
                    self.get_logger().error(f"Rejected nvblox worker batch; last valid map retained: {exc}")
                finally:
                    self.mesh_future = None
            batch = self.pending_source.take()
            if batch is not None:
                self.mesh_future = self.mesh_worker.submit(self.encode_batch, batch)

        def enqueue(self, update: MeshPageUpdate) -> None:
            self.scheduler.enqueue(update)

        def enqueue_snapshot(self) -> None:
            now = time.monotonic()
            if now - self.last_snapshot_time < 1.0:
                return
            self.last_snapshot_time = now
            # Replay current pages plus bounded slot tombstones. Broadcasting a
            # same-epoch clear would erase the map on every connected headset.
            updates = self.committed_snapshot
            if not updates:
                # A worker may be building the first real map. Do not publish
                # a concurrent/partially mutated state as an empty snapshot.
                if self.mesh_future is not None or self.pending_source.blocks or self.pending_source.clear:
                    return
                updates = [self.state.snapshot_clear_update()]
            for update in updates:
                self.enqueue(update)

        def on_snapshot_request(self, request: String) -> None:
            try:
                content = json.loads(request.data)
            except (ValueError, TypeError):
                content = None
            if isinstance(content, dict) and content.get("request") == "pages":
                ids = content.get("ids")
                valid_ids = (isinstance(ids, list) and len(ids) <= args.max_pages and
                             all(isinstance(value, int) and not isinstance(value, bool) for value in ids))
                if not valid_ids:
                    return
                requested = set(ids)
                updates = [page for page in self.committed_snapshot if page.chunk_id in requested] if (
                    str(content.get("epoch")) == self.committed_manifest["epoch"]) else self.committed_snapshot
                for update in updates:
                    self.enqueue(update)
                queued = len(updates)
            else:
                self.enqueue_snapshot()
                queued = self.scheduler.pending_count
            self.get_logger().info(
                f"Snapshot requested ({request.data[:80]!r}); selected {queued} pages, pending={self.scheduler.pending_count}"
            )

        def publish_pending(self) -> None:
            self.advance_source()
            while True:
                update = self.scheduler.pop_ready(time.monotonic())
                if update is None:
                    break
                message = UInt8MultiArray()
                message.data = array("B", update.payload)
                self.publisher.publish(message)
                self.published_pages += 1
                self.published_bytes += len(update.payload)

        def log_status(self) -> None:
            manifest = dict(self.committed_manifest)
            if args.presentation_frame:
                manifest["source_frame_id"] = manifest["frame_id"]
                manifest["frame_id"] = args.presentation_frame
            self.manifest_publisher.publish(String(data=json.dumps(manifest, separators=(",", ":"))))
            age = time.monotonic() - self.last_input_time if self.last_input_time else math.inf
            status = {
                "input": args.input_topic,
                "source_delivery": self.source_delivery,
                "frame": self.committed_manifest["frame_id"],
                "messages": self.received_messages,
                "blocks": self.committed_blocks,
                "pages": sum(page.operation == 0 for page in self.committed_snapshot),
                "triangles": self.committed_manifest["triangles"],
                "source_pending_blocks": len(self.pending_source.blocks),
                "source_pending_bytes": self.pending_source.bytes,
                "encode_worker_active": self.mesh_future is not None,
                "last_encode_seconds": round(self.last_encode_seconds, 4),
                "pending": self.scheduler.pending_count,
                "pending_bytes": self.scheduler.pending_bytes,
                "published": self.published_pages,
                "published_bytes": self.published_bytes,
                "input_age_s": None if not math.isfinite(age) else round(age, 2),
            }
            self.get_logger().info(json.dumps(status, separators=(",", ":")))

    rclpy.init()
    node = NvbloxMeshAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mesh_worker.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
