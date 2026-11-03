"""Real-capture render acceptance on the workstation GPU (not headset FPS)."""

import importlib.util
import os
from pathlib import Path
import sys

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get("HORUS_TEST_REAL_GPU") != "1",
                                reason="requires real map cache, ROS and CUDA")


@pytest.mark.parametrize("dataset,kind,count", [
    ("g1", "mesh", 227954), ("spot", "mesh", 29802),
    ("cow-lady", "points", 643343), ("g1", "points", 3752449),
])
def test_all_recorded_primitives_produce_nonblank_stereo(monkeypatch, dataset, kind, count):
    import rclpy
    tools = Path(__file__).parents[1] / "examples/tools"
    monkeypatch.syspath_prepend(str(tools))
    from real_mapping_data import record
    path = tools / "remote_map_render_agent.py"
    spec = importlib.util.spec_from_file_location("recorded_gpu_agent", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(sys, "argv", [str(path), "--protocol", "v3", "--transport", "ros_compressed",
        "--scene", "recorded_" + kind, "--dataset-path", record(dataset, kind)["path"],
        "--width", "640", "--height", "480", "--depth-width", "320", "--depth-height", "240",
        "--fps", "1", "--render-update-fps", "1"])
    rclpy.init()
    node = None
    try:
        node = module.RemoteMapRenderSource(module.parse_args())
        assert node.source_point_count == count
        assert np.count_nonzero(node.color) > 500
        assert node.color.max() > node.color.min()
        if kind == "mesh":
            assert node._cuda_renderer.face_count == count
            assert node._cuda_renderer._color_batches is not None
        print(f"REAL_GPU dataset={dataset} kind={kind} primitives={count} nonzero={np.count_nonzero(node.color)}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


def test_gpu_interpolates_source_vertex_colors():
    from horus.remote_rendering.nvdiffrast_mesh_renderer import NvdiffrastMeshRenderer
    renderer = NvdiffrastMeshRenderer(
        np.array([[-1, -1, 3], [1, -1, 3], [0, 1, 3]], np.float32),
        np.array([[0, 1, 2]], np.int32), np.full((1, 3), 255, np.uint8),
        vertex_colors=np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]], np.uint8))
    try:
        color, depth, _ = renderer.render_views([[0, 0, 0]], [[0, 0, 0, 1]], 320, 240,
            vertical_fov_deg=60, near_m=.05, far_m=10)
        pixels = color[0][np.isfinite(depth[0])]
        assert len(pixels) > 100
        assert np.unique(pixels, axis=0).shape[0] > 100
        assert np.all(pixels.max(axis=0) > 200)
    finally:
        renderer.close()
