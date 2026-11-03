"""Serve finished G1 meshes and aligned occupancy without reconstruction."""

from array import array
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from g1_recording_registration import TOPIC
from tools.g1_recording_data import CACHE, bundle, load_occupancy, occupancy_path
from tools.real_mapping_data import arrays
from horus.mapping.live_mesh import LiveMeshState
from horus.mapping.mesh_protocol import BlockGeometry
from horus.mapping.mesh_page_scheduler import MeshPageScheduler


def digest(path):
    checksum = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            checksum.update(chunk)
    return checksum.hexdigest()


def prepare(dataset):
    # Optional preparation uses the existing PLY exporter/dependencies, not ROS.
    from tools.prepare_real_mapping_data import export_mesh
    CACHE.mkdir(parents=True, exist_ok=True)
    target = CACHE / (dataset + '-mesh.npz')
    temporary = CACHE / (dataset + '-mesh.partial.npz')
    meta = export_mesh(bundle(dataset) / 'mapping/mesh.ply', temporary,
                       'Completed G1 nvblox export; no reconstruction or triangle removal')
    temporary.replace(target)
    meta['path'] = str(target)
    (CACHE / (dataset + '-mesh.json')).write_text(json.dumps(meta, indent=2) + '\n')
    return meta


def mesh_arrays(dataset):
    metadata = CACHE / (dataset + '-mesh.json')
    if not metadata.is_file():
        raise FileNotFoundError(f'Missing prepared mesh {metadata}; see MAPPING_VISUALIZATION_TESTING.md')
    meta = json.loads(metadata.read_text())
    target, source = Path(meta['path']), bundle(dataset) / 'mapping/mesh.ply'
    if Path(meta['source']) != source or digest(source) != meta['source_sha256'] or digest(target) != meta['sha256']:
        raise ValueError('Prepared mesh does not match this recording; prepare its cache again')
    return arrays(target, mesh=True)


def mesh_state(vertices, colors, faces, page_triangles=4096):
    state = LiveMeshState(frame_id='map')
    # Spatial ordering changes only page membership, not faces or winding.
    centroids = vertices[faces].mean(axis=1)
    cells = np.floor(centroids / 1.6).astype(np.int64)
    order = np.lexsort((cells[:, 2], cells[:, 1], cells[:, 0]))
    for page, start in enumerate(range(0, len(order), page_triangles)):
        indices, inverse = np.unique(faces[order[start:start+page_triangles]].reshape(-1), return_inverse=True)
        geometry = BlockGeometry(tuple(map(tuple, vertices[indices].tolist())),
            tuple(tuple(map(int, color)) + (255,) for color in colors[indices]),
            tuple(map(int, inverse)))
        state.replace(page, geometry)
    return state


def run(dataset, map_mode, ready_file):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String, UInt8MultiArray

    occupancy = load_occupancy(occupancy_path(dataset))
    state = mesh_state(*mesh_arrays(dataset)) if map_mode == 'static' else None
    scheduler = MeshPageScheduler(pages_per_second=30, bytes_per_second=4*1024**2,
        max_pages=1024, max_payload_bytes=1024**2, now=time.monotonic())
    rclpy.init()
    node = rclpy.create_node('horus_g1_completed_maps')
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub = node.create_publisher(OccupancyGrid, TOPIC + '/occupancy', retained)
    last_request = -float('inf')
    if state:
        mesh_pub = node.create_publisher(UInt8MultiArray, TOPIC + '/mesh/indexed', 64)
        manifest_pub = node.create_publisher(String, TOPIC + '/mesh/indexed_manifest', retained)

        def inventory():
            manifest_pub.publish(String(data=json.dumps(state.manifest())))

        def request(msg):
            nonlocal last_request
            if time.monotonic() - last_request < 0.5:
                return
            try:
                data = json.loads(msg.data)
            except ValueError:
                return
            for update in state.repair_updates(data):
                scheduler.enqueue(update)
            last_request = time.monotonic()

        def publish():
            while (update := scheduler.pop_ready(time.monotonic())) is not None:
                mesh_pub.publish(UInt8MultiArray(data=array('B', update.payload)))

        node.create_subscription(String, TOPIC + '/mesh/indexed_request', request, 16)
        node.create_timer(0.01, publish)
        node.create_timer(2, inventory)
        inventory()
    occupancy.header.stamp = node.get_clock().now().to_msg()
    grid_pub.publish(occupancy)
    summary = dict(dataset=dataset, map_mode=map_mode,
        grid_width=occupancy.info.width, grid_height=occupancy.info.height,
        grid_resolution=occupancy.info.resolution, occupancy='saved final snapshot',
        triangles=state.manifest()['triangles'] if state else None,
        pages=state.page_count if state else None)
    Path(ready_file).write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary), flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', choices=('first', 'later'), required=True)
    parser.add_argument('--map', dest='map_mode', choices=('static', 'live'), default='static')
    parser.add_argument('--ready-file', type=Path)
    parser.add_argument('--prepare', action='store_true')
    args = parser.parse_args()
    if args.prepare:
        print(json.dumps(prepare(args.dataset), indent=2))
    elif args.ready_file:
        run(args.dataset, args.map_mode, args.ready_file)
    else:
        parser.error('--ready-file is required for publishing')
