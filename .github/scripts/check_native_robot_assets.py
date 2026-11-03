#!/usr/bin/env python3
"""Optional real-asset qualification. Reads URDF/meshes; never starts ROS or robots."""
import argparse
import base64
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np


def transform(element):
    origin = element.find("origin")
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()] if origin is not None else [0]*3
    r, p, y = [float(v) for v in origin.get("rpy", "0 0 0").split()] if origin is not None else [0]*3
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    matrix = np.eye(4)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                      [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = xyz
    return matrix


def verify(path, mesh_root, binary):
    tree = ET.parse(path).getroot()
    links = {link.get("name"): link for link in tree.findall("link")}
    parents = {j.find("child").get("link"): j for j in tree.findall("joint")}
    root, = set(links)-set(parents)
    poses = {root: np.eye(4)}

    def pose(name):
        if name not in poses:
            joint = parents[name]
            poses[name] = pose(joint.find("parent").get("link")) @ transform(joint)
        return poses[name]

    start = time.monotonic()
    result = subprocess.run([binary, "--source", json.dumps({
        "source": "local", "urdf_path": str(path), "mesh_root": str(mesh_root),
        "base_frame": root, "body_mesh_mode": "runtime_high_mesh",
        "preserve_source_mesh": True, "visual_link_pose_source": "tf",
    })], capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, (path, result.stderr)
    artifact = json.loads(result.stdout)
    payload = artifact["payload"]
    expected_groups = set()
    for name, link in links.items():
        if link.find("visual/geometry/mesh") is None:
            continue
        group = name
        while group in parents and parents[group].get("type") == "fixed":
            group = parents[group].find("parent").get("link")
        expected_groups.add(group)
    assert {v["frame_id"] for v in payload["visual_links"]} == expected_groups
    palette = set()
    for visual in payload["visual_links"]:
        np.testing.assert_allclose(visual["origin_xyz"], pose(visual["frame_id"])[:3, 3], atol=1e-6)
    for joint in payload["joints"]:
        np.testing.assert_allclose(joint["origin_xyz"], pose(joint["child_link"])[:3, 3], atol=1e-6)
    for asset in payload["mesh_assets"]:
        points = np.frombuffer(base64.b64decode(asset["positions_b64"]), dtype="<f4").reshape(-1, 3)
        normals = np.frombuffer(base64.b64decode(asset["normals_b64"]), dtype="<f4").reshape(-1, 3)
        indices = np.frombuffer(base64.b64decode(asset["indices_b64"]), dtype="<u4")
        assert len(points) == asset["vertex_count"] and len(indices) == asset["triangle_count"]*3
        assert np.isfinite(points).all() and np.isfinite(normals).all() and indices.max() < len(points)
        np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1, atol=1e-5)
        np.testing.assert_allclose(points.min(axis=0), asset["bounds_min"], atol=1e-6)
        np.testing.assert_allclose(points.max(axis=0), asset["bounds_max"], atol=1e-6)
        colors = base64.b64decode(asset["colors_b64"])
        assert len(colors) == len(points)*4
        palette.update(zip(*[iter(colors)]*4))
    manifest = artifact["manifest"]
    assert manifest["source_triangle_count"] == manifest["triangle_count"]
    print(json.dumps({"binary": str(binary), "robot": path.stem, "seconds": round(time.monotonic()-start, 3),
                      "links": manifest["link_count"], "visual_groups": len(expected_groups),
                      "triangles": manifest["triangle_count"], "palette_size": len(palette),
                      "encoded_bytes": manifest["mesh_asset_encoded_bytes"],
                      "description_id": manifest["description_id"]}), flush=True)
    return artifact["payload_json"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assets", type=Path)
    parser.add_argument("binaries", nargs="+", type=Path)
    parser.add_argument("--robots", nargs="+", default=["jackal", "go1", "anymal_c", "g1", "h1", "spot"])
    args = parser.parse_args()
    for robot in args.robots:
        reference = None
        for binary in args.binaries:
            payload = verify(args.assets / (robot+".urdf"), args.assets / "meshes_root", binary)
            if reference is not None:
                assert payload == reference, f"{robot}: C++/Rust model payloads differ"
            reference = payload
