#!/usr/bin/env python3
"""Compare native multiview RGB/depth, packed frames and surfels with Python."""
import copy
from dataclasses import asdict
import json
import subprocess
import sys
import numpy as np
from horus.remote_rendering.real_scene import RemoteRenderViewSpec, render_multiview_atlas, unproject_multiview_surfels
from horus.remote_rendering.synthetic_scene import encode_luma_depth

rng = np.random.default_rng(321)
points = rng.uniform([-1.3, -.6, 1.2], [1.6, .9, 5], (321, 3)).astype(np.float32)
colors = rng.integers(0, 256, (321, 3), dtype=np.uint8)
views = (RemoteRenderViewSpec('front', (0, 0, 0), 0, 0, 0, 0, .5, 1),
         RemoteRenderViewSpec('side', (.2, -.1, .3), 22, -11, .5, 0, .5, 1))
o = dict(width=96, height=48, point_radius=1, hole_fill_iterations=2, world_scale=1.0,
         near_base=.3, far_base=20.0, vertical_fov_degrees=52.0)
color, depth = render_multiview_atlas(points, colors, 96, 48, views, point_radius=1,
                                    world_scale=1., near_base=.3, far_base=20.)
surfels, rgb = unproject_multiview_surfels(color, depth, views, columns=12, rows=8)
packed = np.concatenate((color, encode_luma_depth(depth, near_m=.3, far_m=20.)), axis=1)
payload = dict(points=points.tolist(), colors=colors.tolist(), views=[asdict(v) for v in views], options=o)
for binary in sys.argv[1:]:
    def run(p):
        return subprocess.run([binary], input=json.dumps(p), text=True, capture_output=True, timeout=30)
    result = run(payload)
    assert result.returncode == 0, result.stderr
    value = json.loads(result.stdout)
    assert np.array_equal(np.asarray(value['rgb']).reshape(color.shape), color)
    d = np.asarray([float('inf') if v is None else v for v in value['depth']]).reshape(depth.shape)
    assert np.array_equal(np.isfinite(d), np.isfinite(depth))
    np.testing.assert_allclose(d, depth, atol=2e-6, rtol=1e-6)
    np.testing.assert_allclose(value['points'], surfels, atol=3e-6, rtol=1e-6)
    assert np.array_equal(value['colors'], rgb)
    assert np.array_equal(np.asarray(value['packed']).reshape(packed.shape), packed)
    for key, bad in [('width', 0), ('height', 100000), ('world_scale', -1), ('point_radius', 999)]:
        invalid = copy.deepcopy(payload)
        invalid['options'][key] = bad
        assert run(invalid).returncode != 0
    print(json.dumps(dict(binary=binary, passed=True, surfels=len(surfels), points=len(points))), flush=True)
