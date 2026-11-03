#!/usr/bin/env python3
"""Independent scene transform/texture/PLY qualification for both native clients."""
import argparse
import base64
import io
import json
from pathlib import Path
import struct
import subprocess
import tempfile

import numpy as np
from PIL import Image


def gltf(root, image):
    vertices = np.asarray([[0, 0, 2], [1, 0, 2], [0, 1, 2]], dtype="<f4")
    uvs = np.asarray([[0, 0], [1, 0], [0, 1]], dtype="<f4")
    indices = np.asarray([0, 1, 2], dtype="<u2")
    blob = vertices.tobytes() + uvs.tobytes() + indices.tobytes()
    (root / "mesh.bin").write_bytes(blob)
    payload = {
        "asset": {"version": "2.0"}, "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"translation": [10, 2, 0], "children": [1, 2]},
                  {"mesh": 0, "translation": [1, 0, 0]},
                  {"mesh": 0, "translation": [-1, 0, 0], "scale": [-1, 1, 1]}],
        "buffers": [{"uri": "mesh.bin", "byteLength": len(blob)}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": 36},
                        {"buffer": 0, "byteOffset": 36, "byteLength": 24},
                        {"buffer": 0, "byteOffset": 60, "byteLength": 6}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": 3,
                       "type": "VEC3", "min": [0, 0, 2], "max": [1, 1, 2]},
                      {"bufferView": 1, "componentType": 5126, "count": 3, "type": "VEC2"},
                      {"bufferView": 2, "componentType": 5123, "count": 3, "type": "SCALAR"}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "TEXCOORD_0": 1},
                                     "indices": 2, "material": 0}]}],
        "materials": [{"pbrMetallicRoughness": {"baseColorTexture": {"index": 0},
                                               "baseColorFactor": [1, 1, 1, 1]}}],
        "textures": [{"source": 0}], "images": [{"uri": image}],
    }
    path = root / "scene.gltf"
    path.write_text(json.dumps(payload))
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("binaries", nargs="+")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="horus-native-scenes-") as temp:
        root = Path(temp)
        pixels = np.asarray([[[255, 0, 0], [0, 255, 0]],
                             [[0, 0, 255], [255, 255, 255]]], np.uint8)
        reference = None
        for codec in ["PNG", "JPEG"]:
            encoded = io.BytesIO()
            Image.fromarray(pixels).save(encoded, format=codec, quality=100, subsampling=0)
            decoded = np.asarray(Image.open(io.BytesIO(encoded.getvalue())).convert("RGB"))
            for embedded in [False, True]:
                name = f"texture.{codec.lower()}"
                (root / name).write_bytes(encoded.getvalue())
                uri = f"data:image/{codec.lower()};base64," + base64.b64encode(encoded.getvalue()).decode() if embedded else name
                path = gltf(root, uri)
                results = []
                for binary in args.binaries:
                    result = subprocess.run([binary, str(path)], capture_output=True, text=True, timeout=30)
                    assert result.returncode == 0, result.stderr
                    scene = json.loads(result.stdout)
                    vertices, faces = np.asarray(scene["vertices"]), np.asarray(scene["faces"])
                    assert len(vertices) == 6 and len(faces) == 2, scene
                    expected = np.asarray([[11, 2, 2], [12, 2, 2], [11, 3, 2],
                                           [9, 2, 2], [8, 2, 2], [9, 3, 2]])
                    assert sorted(map(tuple, vertices)) == sorted(map(tuple, expected))
                    for face in faces:
                        a, b, c = vertices[face]
                        assert np.cross(b-a, c-a)[2] > 0, "Mirrored node reversed winding"
                    atlas = np.asarray(scene["atlas"], np.uint8).reshape(scene["height"], scene["width"], 3)
                    rects = scene["texture_rects"]
                    assert rects[0] == rects[1], "Instanced texture was duplicated"
                    x, y, w, h = rects[0]
                    np.testing.assert_array_equal(atlas[y:y+h, x:x+w], decoded)
                    np.testing.assert_array_equal(atlas[y-1, x:x+w], decoded[0])
                    assert scene["face_colors"] == [[255, 255, 255]] * 2
                    results.append(scene)
                assert all(r == results[0] for r in results)
                reference = results[0]
        for codec in ["BMP", "TGA", "GIF", "PPM", "TIFF", "WEBP"]:
            image = root / ("texture." + codec.lower())
            Image.fromarray(pixels).save(image, format=codec, lossless=True)
            decoded = np.asarray(Image.open(image).convert("RGB"))
            (root / "material.mtl").write_text(f"newmtl surface\nKd 1 1 1\nmap_Kd {image.name}\n")
            obj = root / "material.obj"
            obj.write_text("mtllib material.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\n"
                           "vt 0 0\nvt 1 0\nvt 0 1\nusemtl surface\nf 1/1 2/2 3/3\n")
            for binary in args.binaries:
                result = subprocess.run([binary, str(obj)], capture_output=True, text=True, timeout=30)
                assert result.returncode == 0, (codec, result.stderr)
                scene = json.loads(result.stdout)
                atlas = np.asarray(scene["atlas"], np.uint8).reshape(scene["height"], scene["width"], 3)
                x,y,w,h = scene["texture_rects"][0]
                np.testing.assert_array_equal(atlas[y:y+h,x:x+w], decoded, err_msg=codec)
            image.write_bytes(b"corrupt texture")
            for binary in args.binaries:
                result = subprocess.run([binary,str(obj)],capture_output=True,text=True,timeout=30)
                assert result.returncode != 0, "Remote scene must not silently replace corrupt material"
        for endian, name in [("<", "binary_little_endian"), (">", "binary_big_endian")]:
            ply = root / "points.ply"
            header = f"ply\nformat {name} 1.0\nelement vertex 2\nproperty double x\nproperty double y\nproperty double z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
            ply.write_bytes(header.encode() + struct.pack(endian+"dddBBB", 1, 2, 3, 255, 4, 5)
                            + struct.pack(endian+"dddBBB", -4, 5, 6, 1, 200, 3))
            for binary in args.binaries:
                result = subprocess.run([binary, str(ply)], capture_output=True, text=True, timeout=30)
                assert result.returncode == 0, result.stderr
                scene = json.loads(result.stdout)
                assert scene["vertices"] == [[1, 2, 3], [-4, 5, 6]]
                assert scene["point_colors"] == [[255, 4, 5], [1, 200, 3]], scene
                assert not scene["faces"]
        assert reference
        print(json.dumps({"result": "passed", "languages": len(args.binaries),
                          "texture_cases": 10, "ply_endians": 2,
                          "node_transforms": True, "mirrored_winding": True,
                          "texture_pixels_and_padding": True, "texture_deduplication": True}))


if __name__ == "__main__":
    main()
