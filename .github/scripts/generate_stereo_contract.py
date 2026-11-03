#!/usr/bin/env python3
"""Cross-language pixel contracts for CPU projection and v3 stereo carriers."""
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np
from horus.remote_rendering.real_scene import render_colored_points
from horus.remote_rendering.v3_renderer import render_stereo_frame


def build():
    cases = []
    for name, width, height, strip, marker, codec in (
        ("color", 8, 4, 0, 0, "png"),
        ("depth", 8, 4, 3, 0, "png"),
        ("marked", 192, 2, 1, 8, "h264"),
    ):
        rgb = (np.arange(2 * height * width * 3) % 256).astype(np.uint8).reshape(2, height, width, 3)
        depth = np.resize(np.array([.25, .5, 1, 2, 4, 8, np.inf, np.nan], dtype=np.float32), (2, height, width))

        class Renderer:
            def render_views(self, *args, **kwargs):
                return rgb, depth, 0.0

        class Pose:
            position = (0, 0, 0)
            rotation = (0, 0, 0, 1)

        with patch("horus.remote_rendering.v3_renderer.time.monotonic_ns", side_effect=[10, 20, 30]):
            frame = render_stereo_frame(Renderer(), [Pose(), Pose()], [(1, 1, 0, 0)] * 2,
                state=None, sequence=42, session_id="native", workspace_revision=3,
                predicted_display_time_ns=100, width=width, height=height, vertical_fov_deg=90,
                near_m=.25, far_m=8, point_radius=0, codec=codec,
                visibility_mask_height=strip, frame_marker_height=marker)
        cases.append({"name": name, "width": width, "height": height, "strip": strip, "marker": marker,
            "codec": codec, "rgb": rgb.flatten().tolist(),
            "depth": [float(d) if np.isfinite(d) else None for d in depth.flatten()],
            "rgba_hex": frame.stereo_rgba.tobytes().hex(), "video_hex": frame.video_rgb.tobytes().hex(),
            "visibility": frame.visibility_fraction, "metadata": frame.metadata.to_dict()})
    points = np.array([[-.45, .2, 2], [0, 0, 2], [.48, -.1, 2], [0, 0, 3], [1, 1, -1]], dtype=np.float32)
    colors = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0], [255, 255, 255]], dtype=np.uint8)
    views = [render_colored_points(points, colors, 32, 24, camera_position=np.array([x,0,0]),
                                  camera_pitch_deg=0, vertical_fov_deg=90, near_m=.25, far_m=8)
             for x in [-.1, .1]]
    return {"cases": cases, "points": points.tolist(), "colors": colors.tolist(),
            "cpu_rgb_hex": b"".join(v[0].tobytes() for v in views).hex(),
            "cpu_depth": [float(v) if np.isfinite(v) else None for r in views for v in r[1].flatten()]}


if __name__ == "__main__":
    path = Path(__file__).resolve().parents[2] / "contracts/fixtures/stereo_render.json"
    path.write_text(json.dumps(build(), indent=2) + "\n")
