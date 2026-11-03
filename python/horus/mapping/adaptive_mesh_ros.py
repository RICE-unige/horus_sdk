"""Independent adaptive HORUS views of any original HMSH v2 mesh source.

The source remains untouched. Each headset requests regions and a bounded draw
budget; its own indexed stream replaces spatial tiles, not the shared map.
"""
from __future__ import annotations

import argparse
from array import array
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import json
import hashlib
import math
import re
from pathlib import Path
import os
import tempfile
import time

from .mesh_page_scheduler import MeshPageScheduler
from .mesh_protocol import HMSH_OPERATION_CLEAR, decode_live_header


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--output-topic", required=True)
    parser.add_argument("--frame-id", required=True)
    parser.add_argument("--save-source", type=Path, help="Fresh output folder for original fine pages at shutdown")
    parser.add_argument("--scene-status-topic", default="", help="Optional calibrated visual-datum status for saved replay placement")
    parser.add_argument("--source-floor-z", type=float, help="Explicit source-frame visual datum for generic saved-map replay")
    parser.add_argument("--max-publish-mib-per-second", type=float, default=4)
    args = parser.parse_args(argv)
    if any(not topic.startswith("/") or topic.endswith("/") for topic in (args.input_topic, args.output_topic)):
        parser.error("input/output must be absolute base topics without trailing slashes")
    if args.input_topic == args.output_topic or not args.frame_id.strip():
        parser.error("distinct topics and an explicit source coordinate frame are required")
    if not math.isfinite(args.max_publish_mib_per_second) or not .1 <= args.max_publish_mib_per_second <= 32:
        parser.error("publish rate must be 0.1..32 MiB/s")
    if args.source_floor_z is not None and not math.isfinite(args.source_floor_z):
        parser.error("source datum must be finite")
    return args


def save_original_pages(store, destination, frame_id, expected=None, source_floor_z=None):
    """Atomic, no-overwrite original snapshot, independent of GPU mesh exporters."""
    destination = Path(destination).expanduser().resolve()
    if destination.exists():
        raise FileExistsError(f"Original snapshot already exists: {destination}")
    if not store.pages:
        raise ValueError("No original geometry received; refusing to label an empty save as a map")
    destination.parent.mkdir(parents=True, exist_ok=True)
    pages = list(store.pages.values())
    inventory = {page.chunk_id: (page.source_epoch, page.source_revision) for page in pages}
    expected_pages = expected.get("pages", []) if expected else []
    expected_live = {page["chunk_id"] for page in expected_pages if page.get("operation") == 0}
    verified = bool(expected) and expected_live == set(inventory) and all(
        inventory.get(page["chunk_id"], (-1, -1)) >= (int(expected["epoch"]), int(page["revision"]))
        for page in expected_pages if page.get("operation") == 0)
    with tempfile.TemporaryDirectory(prefix="." + destination.name + "-", dir=destination.parent) as temporary:
        root = Path(temporary)
        (root / "pages").mkdir()
        records = []
        files = []
        checksums = {}
        for page in pages:
            name = f"{page.chunk_id:08d}.hmsh"
            (root / "pages" / name).write_bytes(page.fine)
            checksums[name] = hashlib.sha256(page.fine).hexdigest()
            records.append({"chunk_id": page.chunk_id, "revision": str(page.source_revision), "operation": 0})
            files.append({"file": "pages/" + name, "sha256": checksums[name], "chunk_id": page.chunk_id,
                "revision": str(page.source_revision), "operation": 0, "vertex_count": page.fine_vertices,
                "triangle_count": page.fine_triangles, "bytes": len(page.fine)})
        record = {"schema": "horus.mesh_fixture.v1", "original_geometry": True,
            "input_inventory_verified": verified, "source_expected_manifest": expected,
            "original_manifest": {"version": 1, "epoch": str(store.source_epoch), "frame_id": frame_id,
                "pages": records, "triangles": sum(page.fine_triangles for page in pages)},
            "pages": files, "triangles": sum(page.fine_triangles for page in pages),
            "vertices": sum(page.fine_vertices for page in pages), "payload_bytes": sum(len(page.fine) for page in pages),
            "workspace_scale": .1, "source_floor_z_m": source_floor_z,
            "replay_requires_datum": source_floor_z is None,
            "geometry": "Original fine source; no LOD or coordinate changes", "sha256": checksums}
        (root / "manifest.json").write_text(json.dumps(record, indent=2) + "\n")
        os.rename(root, destination)
    return record


def client_topic_token(client):
    """Keep request identities opaque while producing a legal ROS namespace token."""
    if not isinstance(client, str) or re.fullmatch(r"[a-f0-9]{32}", client) is None:
        raise ValueError("invalid adaptive client ID")
    return "c_" + client if client[0].isdigit() else client


def validate_request(text):
    if not isinstance(text, str) or len(text) > 32768:
        raise ValueError("oversized adaptive request")
    request = json.loads(text)
    if not isinstance(request, dict) or request.get("version") != 1:
        raise ValueError("unsupported adaptive request")
    client = request.get("client_id", "")
    client_topic_token(client)
    seq = request.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        raise ValueError("invalid adaptive sequence")
    if not isinstance(request.get("frame_id"), str) or not request["frame_id"]:
        raise ValueError("source frame required")
    for name, low, high in (("triangle_budget", 1000, 10_000_000), ("memory_budget", 16 << 20, 384 << 20)):
        value = request.get(name)
        if not isinstance(value, int) or isinstance(value, bool) or not low <= value <= high:
            raise ValueError(f"invalid adaptive {name}")
    regions = request.get("regions")
    unresolved = request.get("unresolved_regions", [])
    if not isinstance(unresolved, list) or len(unresolved) > 16 or any(not isinstance(value, str) or len(value) > 128 for value in unresolved):
        raise ValueError("invalid unresolved-interest report")
    if not isinstance(regions, list) or len(regions) > 16:
        raise ValueError("at most 16 interest areas")
    identifiers = set()
    for region in regions:
        if not isinstance(region, dict) or not isinstance(region.get("id"), str) or region["id"] in identifiers:
            raise ValueError("interest IDs must be unique")
        identifiers.add(region["id"])
        if region.get("shape") not in ("box", "sphere"):
            raise ValueError("interest shape must be box or sphere")
        for name, length in (("center", 3), ("size", 3), ("rotation", 4)):
            values = region.get(name, [0, 0, 0, 1] if name == "rotation" else None)
            if not isinstance(values, list) or len(values) != length or not all(
                isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in values):
                raise ValueError(f"invalid interest {name}")
        if not all(.1 <= size <= 30 for size in region["size"]):
            raise ValueError("interest size must be 0.1..30 m")
        if region["shape"] == "sphere" and len(set(region["size"])) != 1:
            raise ValueError("sphere diameters must be equal")
        if sum(v * v for v in region.get("rotation", [0, 0, 0, 1])) < 1e-12:
            raise ValueError("interest rotation must be nonzero")
        for name, default, low, high in (("priority", 1, 0, 100), ("min_fraction", .25, 0, 1)):
            value = region.get(name, default)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"invalid interest {name}")
    return request


def create_node(args):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String, UInt8MultiArray
    from .adaptive_mesh import AdaptiveMeshStore, ensure_native

    ensure_native()

    class AdaptiveMeshServer(Node):
        def __init__(self):
            super().__init__("horus_adaptive_mesh")
            self.store = AdaptiveMeshStore()
            self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="horus-adaptive")
            self.future = None
            self.pending = OrderedDict()
            self.pending_bytes = 0
            self.pending_requests = {}
            self.requests = {}
            self.views = {}
            self.repairs = {}
            self.source_epoch = -1
            self.source_versions = {}
            self.worker_versions = {}
            self.worker_epoch = -1
            self.committed_epoch = -1
            self.source_manifest = None
            self.source_floor_z = args.source_floor_z
            self.last_inventory = self.last_repair = self.last_work = 0.0
            self.last_warning = -math.inf
            self.last_source_error = ""
            self.cleanup_pending = False
            self.byte_credit = 0.0
            self.last_credit = time.monotonic()
            self.round_robin = 0
            self.rejected = self.sent = self.sent_bytes = self.received_requests = 0
            reliable = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE)
            self.source = self.create_subscription(UInt8MultiArray, args.input_topic + "/indexed", self.receive, reliable)
            self.inventory = self.create_subscription(String, args.input_topic + "/indexed_manifest", self.receive_inventory,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.source_request = self.create_publisher(String, args.input_topic + "/indexed_request", 4)
            self.request = self.create_subscription(String, args.output_topic + "/adaptive_request", self.receive_request, 16)
            self.scene_status = self.create_subscription(String, args.scene_status_topic, self.receive_scene, 1) if args.scene_status_topic else None
            self.status = self.create_publisher(String, args.output_topic + "/status", 1)
            self.timer = self.create_timer(.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info(f"Adaptive map: original {args.input_topic}; independent views {args.output_topic}; "
                "original geometry retained, colour-aware 3 cm context target; no coverage clipping")

        def warn(self, message):
            if time.monotonic() - self.last_warning > 2:
                self.get_logger().warning(str(message))
                self.last_warning = time.monotonic()

        def receive(self, message):
            payload = bytes(message.data)
            try:
                header = decode_live_header(payload)
                if header.epoch < self.source_epoch:
                    return
                if header.epoch > self.source_epoch:
                    self.pending.clear()
                    self.pending_bytes = 0
                    self.source_epoch = header.epoch
                key = -1 if header.operation == HMSH_OPERATION_CLEAR else header.chunk_id
                previous = self.pending.get(key)
                if previous is not None and decode_live_header(previous).revision >= header.revision:
                    return
                if len(self.pending) >= 1025 and key not in self.pending or (
                    self.pending_bytes + len(payload) - len(previous or b"") > 64 << 20):
                    raise ValueError("adaptive source queue full; last valid map retained, inventory repair will retry")
                self.pending[key] = payload
                self.pending_bytes += len(payload) - len(previous or b"")
            except (ValueError, OverflowError) as error:
                self.rejected += 1
                self.warn(error)

        def receive_inventory(self, message):
            try:
                if len(message.data) > 256 << 10:
                    return
                manifest = json.loads(message.data)
                if not isinstance(manifest, dict) or manifest.get("version") != 1 or not isinstance(manifest.get("pages"), list) or len(manifest["pages"]) > 1024:
                    return
                int(manifest["epoch"])
                ids = set()
                for page in manifest["pages"]:
                    chunk = page["chunk_id"]
                    if not isinstance(chunk, int) or not 0 <= chunk <= 0x3fffffff or chunk in ids or page.get("operation") not in (0, 1):
                        return
                    ids.add(chunk)
                    if int(page["revision"]) < 0:
                        return
                if self.source_manifest is not None and int(manifest["epoch"]) < int(self.source_manifest["epoch"]):
                    return
                self.source_manifest = manifest
            except (ValueError, KeyError, TypeError):
                self.rejected += 1

        def receive_scene(self, message):
            try:
                if len(message.data) > 8192:
                    return
                status = json.loads(message.data)
                datum = status.get("source_floor_z_m")
                if status.get("source_frame") == args.frame_id and type(datum) in (float, int) and math.isfinite(datum):
                    self.source_floor_z = float(datum)
            except (ValueError, TypeError):
                return

        def receive_request(self, message):
            try:
                request = validate_request(message.data)
                client = request["client_id"]
                now = time.monotonic()
                previous = self.requests.get(client)
                if previous is not None and request["seq"] <= previous[0]["seq"]:
                    return
                if client not in self.views:
                    if len(self.views) >= 4:
                        raise ValueError("four active adaptive operators already present")
                    base = args.output_topic + "/clients/" + client_topic_token(client)
                    self.views[client] = {
                        "publisher": self.create_publisher(UInt8MultiArray, base + "/indexed", 8),
                        "manifest": self.create_publisher(String, base + "/indexed_manifest",
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                        "repair": self.create_subscription(String, base + "/indexed_request",
                            lambda msg, identity=client: self.receive_repair(identity, msg), 4),
                        "scheduler": MeshPageScheduler(pages_per_second=60,
                            bytes_per_second=args.max_publish_mib_per_second * 1024 * 1024,
                            max_pages=1024, max_payload_bytes=16 << 20, now=now),
                        "inventory": None,
                        "last_manifest": 0.0,
                    }
                self.requests[client] = (request, now)
                self.pending_requests[client] = request
                self.received_requests += 1
            except (ValueError, TypeError) as error:
                self.rejected += 1
                self.warn(error)

        def receive_repair(self, client, message):
            try:
                if len(message.data) <= 32768:
                    request = json.loads(message.data)
                    if isinstance(request, dict):
                        self.repairs[client] = request
            except (ValueError, TypeError):
                self.rejected += 1

        def work(self, pages, requests, repairs, now):
            errors = []
            changed = False
            for payload in pages:
                try:
                    accepted = self.store.ingest(payload)
                    if not accepted and not self.store.last_rejection.startswith("stale source"):
                        raise ValueError(self.store.last_rejection or "adaptive source page rejected")
                    changed |= accepted
                    header = decode_live_header(payload)
                    if header.epoch > self.worker_epoch:
                        self.worker_versions.clear()
                        self.worker_epoch = header.epoch
                    if header.epoch == self.worker_epoch and header.operation != HMSH_OPERATION_CLEAR:
                        self.worker_versions[header.chunk_id] = max(self.worker_versions.get(header.chunk_id, (-1, -1)),
                            (header.epoch, header.revision))
                except (ValueError, RuntimeError, MemoryError, OverflowError) as error:
                    errors.append(str(error))
            results = {}
            for client, request in requests.items():
                try:
                    regions = request["regions"] if request["frame_id"].lstrip("/") == args.frame_id.lstrip("/") else []
                    updates, manifest, status = self.store.request_view(client, regions,
                        request["triangle_budget"], request["memory_budget"], now=now)
                    manifest["frame_id"] = args.frame_id
                    manifest["adaptive"] = status
                    status["frame_mismatch"] = not regions and bool(request["regions"])
                    status["stale_regions"] = [region["id"] for region in regions if region.get("stale") is True]
                    status["unresolved_regions"] = request.get("unresolved_regions", [])
                    results[client] = [updates, manifest]
                except (ValueError, RuntimeError, MemoryError, OverflowError) as error:
                    errors.append(str(error))
            for client, request in repairs.items():
                if client in results:
                    results[client][0].extend(self.store.snapshot(client, request))
            self.store.expire_clients(now)
            return results, dict(self.worker_versions), errors, self.worker_epoch

        def tick(self):
            now = time.monotonic()
            for client, (_, seen) in list(self.requests.items()):
                if now - seen <= 15:
                    continue
                state = self.views.pop(client)
                self.destroy_subscription(state["repair"])
                self.destroy_publisher(state["publisher"])
                self.destroy_publisher(state["manifest"])
                self.requests.pop(client, None)
                self.pending_requests.pop(client, None)
                self.repairs.pop(client, None)
                self.cleanup_pending = True
            if self.future is not None and self.future.done():
                try:
                    results, self.source_versions, errors, self.committed_epoch = self.future.result()
                    for error in errors:
                        self.rejected += 1
                        self.warn(error)
                    self.last_source_error = errors[-1] if errors else self.last_source_error
                    for client, (updates, manifest) in results.items():
                        if client not in self.views:
                            continue
                        state = self.views[client]
                        manifest["adaptive"]["source_incomplete"] = bool(self.last_source_error)
                        for update in updates:
                            state["scheduler"].enqueue(update)
                        state["inventory"] = manifest
                        if now - state["last_manifest"] >= .5:
                            state["manifest"].publish(String(data=json.dumps(manifest, separators=(",", ":"))))
                            state["last_manifest"] = now
                except Exception as error:
                    self.rejected += 1
                    self.last_source_error = str(error)
                    self.warn(f"adaptive worker rejected update: {error}")
                self.future = None
            if self.future is None and (self.pending or self.pending_requests or self.repairs or self.cleanup_pending) and now - self.last_work >= .1:
                pages = []
                while self.pending and len(pages) < 16:
                    _, payload = self.pending.popitem(last=False)
                    self.pending_bytes -= len(payload)
                    pages.append(payload)
                requests = {client: request for client, (request, _) in self.requests.items()}
                repairs, self.repairs = self.repairs, {}
                self.pending_requests.clear()
                self.cleanup_pending = False
                self.future = self.worker.submit(self.work, pages, requests, repairs, now)
                self.last_work = now
            # One aggregate byte ceiling across clients, not N independent bursts.
            rate = args.max_publish_mib_per_second * 1024 * 1024
            self.byte_credit = min(rate * .1, self.byte_credit + (now - self.last_credit) * rate)
            self.last_credit = now
            clients = list(self.views.values())
            for offset in range(len(clients)):
                state = clients[(self.round_robin + offset) % len(clients)]
                if state["publisher"].get_subscription_count() == 0:
                    continue
                # Scheduler provides per-page credit; aggregate allows at most one oversized page per turn.
                if self.byte_credit <= 0:
                    break
                update = state["scheduler"].pop_ready(now)
                if update is not None:
                    state["publisher"].publish(UInt8MultiArray(data=array("B", update.payload)))
                    self.byte_credit -= len(update.payload)
                    self.sent += 1
                    self.sent_bytes += len(update.payload)
            self.round_robin += 1
            if now - self.last_repair >= 2:
                self.last_repair = now
                missing = []
                if self.source_manifest is not None:
                    epoch = int(self.source_manifest["epoch"])
                    for page in self.source_manifest.get("pages", []):
                        if self.source_versions.get(page["chunk_id"], (-1, -1)) < (epoch, int(page["revision"])):
                            missing.append(page["chunk_id"])
                reset_missing = self.source_manifest is not None and int(self.source_manifest["epoch"]) != self.committed_epoch
                if not missing and not reset_missing and self.source_manifest is not None and not self.pending:
                    self.last_source_error = ""
                if missing or self.source_manifest is None or reset_missing:
                    repair = {"request": "pages", "epoch": str(self.source_manifest["epoch"]), "ids": missing} if self.source_manifest and not reset_missing else {"request": "snapshot"}
                    self.source_request.publish(String(data=json.dumps(repair)))
            if now - self.last_inventory >= 2:
                self.last_inventory = now
                for state in self.views.values():
                    if state["inventory"] is not None:
                        state["manifest"].publish(String(data=json.dumps(state["inventory"], separators=(",", ":"))))
                # Ingest replaces the page dictionary, so this is one stable source inventory.
                source_pages = self.store.pages
                self.status.publish(String(data=json.dumps({"clients": len(self.views),
                    "received_requests": self.received_requests,
                    "last_request_age_seconds": min((now - seen for _, seen in self.requests.values()), default=None),
                    "source_pages": len(source_pages),
                    "source_triangles": sum(page.fine_triangles for page in source_pages.values()),
                    "context_triangles": sum(page.coarse_triangles for page in source_pages.values()),
                    "waiting_for_subscribers": sum(state["publisher"].get_subscription_count() == 0 for state in self.views.values()),
                    "pending_pages": len(self.pending),
                    "pending_bytes": self.pending_bytes, "sent_pages": self.sent, "sent_bytes": self.sent_bytes,
                    "rejected": self.rejected, "source": args.input_topic, "frame_id": args.frame_id})))

        def destroy_node(self):
            # Drain accepted latest source work before taking a CPU-only checkpoint.
            try:
                if self.future is not None:
                    self.future.result(timeout=60)
                if self.pending:
                    pages = list(self.pending.values())
                    self.worker.submit(self.work, pages, {}, {}, time.monotonic()).result(timeout=60)
                if args.save_source is not None:
                    record = save_original_pages(self.store, args.save_source, args.frame_id, self.source_manifest, self.source_floor_z)
                    message = (f"Original mesh saved: {args.save_source}; "
                        f"input inventory verified={record['input_inventory_verified']}")
                    if rclpy.ok():
                        self.get_logger().info(message)
                    else:
                        print(message, flush=True)
            except (ValueError, RuntimeError, OSError, TimeoutError) as error:
                message = f"Original mesh checkpoint failed: {error}"
                if rclpy.ok():
                    self.get_logger().error(message)
                else:
                    print(message, flush=True)
            finally:
                self.worker.shutdown(wait=True, cancel_futures=True)
            return super().destroy_node()

    return AdaptiveMeshServer()


def main(argv=None):
    import rclpy
    args = arguments(argv)
    rclpy.init()
    node = create_node(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
