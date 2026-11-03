#!/usr/bin/env python3
"""Compare native original-3DGS import/render against the Python public path."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from plyfile import PlyData, PlyElement
from horus.remote_rendering.gaussian_renderer import load_gaussian_splat_ply, GaussianSplatRenderer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("binaries", nargs="+")
    parser.add_argument("--plugin")
    args = parser.parse_args()
    fixture = json.loads((Path(__file__).parents[2] / "contracts/fixtures/remote_protocol.json").read_text())
    view = fixture["view"]
    for side, x in [("left_eye", -0.03), ("right_eye", 0.03)]:
        view[side]["position"] = {"x": x, "y": 0.1, "z": 0.0}
        view[side]["rotation"] = {"x": 0., "y": 0.08, "z": 0., "w": float(np.sqrt(1 - .08**2))}
    rng = np.random.default_rng(20261014)
    cases = 0
    maximum_depth_error = 0.0
    maximum_rgb_error = 0
    with tempfile.TemporaryDirectory(prefix="horus-native-gaussian-") as temp:
        root = Path(temp)
        for degree in range(5):
            k = (degree + 1)**2
            names = ["x", "y", "z", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3", "opacity", "f_dc_0", "f_dc_1", "f_dc_2"]
            names += [f"f_rest_{i}" for i in range(3 * (k-1))]
            data = np.zeros(64, dtype=[(n, "f4" if degree % 2 else "f8") for n in names])
            for n in names:
                data[n] = rng.normal(size=64) * .3
            data["z"] += 2
            for n in ["scale_0", "scale_1", "scale_2"]:
                data[n] -= 2.8
            for n in ["rot_0", "rot_1", "rot_2", "rot_3"]:
                data[n][0] = 0
            for endian, ascii_file in [("<", True), ("<", False), (">", False)]:
                path = root / "gaussians.ply"
                PlyData([PlyElement.describe(data, "vertex")], text=ascii_file, byte_order=endian).write(path)
                for canonical in [False, True]:
                    options = {"world_scale": 1.7, "canonicalize_y_down": canonical, "view": view, "width": 256, "height": 192}
                    config = root / "options.json"
                    config.write_text(json.dumps(options))
                    source = load_gaussian_splat_ply(path, world_scale=1.7, canonicalize_y_down=canonical)
                    expected = {"means": source.means, "scales": source.scales, "quaternions": source.quaternions,
                                "opacities": source.opacities, "spherical_harmonics": source.spherical_harmonics.reshape(-1, 3),
                                "world_bounds_min": source.world_bounds_min, "world_bounds_max": source.world_bounds_max,
                                "source_from_world_translation": source.transform.source_from_world_translation}
                    render = args.plugin and not ascii_file and endian == "<"
                    if render:
                        poses = [view["left_eye"], view["right_eye"]]
                        positions = [[e["position"][v] for v in "xyz"] for e in poses]
                        rotations = [[e["rotation"][v] for v in "xyzw"] for e in poses]
                        projections = []
                        for eye in poses:
                            f = eye["fov"]
                            projections.append((2/(f["right"]-f["left"]), 2/(f["up"]-f["down"]),
                                                -(f["right"]+f["left"])/(f["right"]-f["left"]),
                                                -(f["up"]+f["down"])/(f["up"]-f["down"])))
                        with GaussianSplatRenderer(source) as renderer:
                            rgb, depth, _ = renderer.render_views(positions, rotations, 256, 192, vertical_fov_deg=90,
                                                                near_m=.05, far_m=100, projections=projections)
                    for binary in args.binaries:
                        output, prefix = root / "out.json", root / "frame"
                        command = [binary, str(path), str(config), str(output)]
                        if render:
                            command += [args.plugin, str(prefix)]
                        result = subprocess.run(command, text=True, capture_output=True, timeout=120)
                        assert result.returncode == 0, result.stderr
                        actual = json.loads(output.read_text())
                        assert actual["sh_degree"] == degree and actual["discarded_nonfinite"] == 0
                        for key, value in expected.items():
                            np.testing.assert_allclose(actual[key], value, atol=3e-6, rtol=3e-6, err_msg=key)
                        if render:
                            native_rgb = np.fromfile(str(prefix)+".rgb", np.uint8).reshape(rgb.shape)
                            native_depth = np.fromfile(str(prefix)+".depth", "<f4").reshape(depth.shape)
                            np.testing.assert_allclose(native_rgb, rgb, atol=1, rtol=0)
                            maximum_rgb_error = max(maximum_rgb_error, int(np.abs(native_rgb.astype(int)-rgb.astype(int)).max()))
                            np.testing.assert_array_equal(np.isfinite(native_depth), np.isfinite(depth))
                            mask = np.isfinite(depth)
                            errors = np.abs(native_depth[mask] - depth[mask])
                            if errors.size:
                                maximum_depth_error = max(maximum_depth_error, float(errors.max()))
                                # Independent float32 camera/normalization arithmetic can change
                                # alpha cutoffs at a few grazing pixels; constrain both tails and mean.
                                assert errors.max() <= .002 and errors.mean() <= 1e-5, (errors.max(), errors.mean())
                                assert np.quantile(errors, .999) <= 1e-4
                    cases += 1
    print(json.dumps({"result": "passed", "cases": cases, "languages": len(args.binaries),
                      "SH_degrees": [0,1,2,3,4], "original_geometry": True,
                      "python_gpu_pixel_depth_comparison": bool(args.plugin),
                      "maximum_depth_error_m": maximum_depth_error,
                      "maximum_rgb_error_8bit": maximum_rgb_error}))


if __name__ == "__main__":
    main()
