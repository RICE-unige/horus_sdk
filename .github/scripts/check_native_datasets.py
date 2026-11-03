#!/usr/bin/env python3
"""Cross-language real-data preparation, chunking, alignment and LOD contracts."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import numpy as np
from plyfile import PlyData, PlyElement
from horus.remote_rendering.real_scene import (load_colored_vertex_ply, prepare_cow_lady_points,
    prepare_eth3d_remote_scene, iter_eth3d_remote_scene_chunks)
from horus.remote_rendering.scene_catalog import ADVANCED_REMOTE_SCENES


def main():
    p=argparse.ArgumentParser()
    p.add_argument("binaries", nargs="+")
    a=p.parse_args()
    rng=np.random.default_rng(29)
    with tempfile.TemporaryDirectory(prefix="horus-native-datasets-") as d:
        root=Path(d)
        records=np.zeros(257,dtype=[("x","f4"),("y","f4"),("z","f4"),("red","u1"),("green","u1"),("blue","u1")])
        for n in "xyz":records[n]=rng.normal(size=len(records))
        for n in ("red","green","blue"):records[n]=rng.integers(0,256,len(records))
        records["x"][7]=np.nan
        for order in ("<",">"):
            PlyData([PlyElement.describe(records,"vertex")],byte_order=order).write(root/("little.ply" if order=="<" else "big.ply"))
        raw,colors=load_colored_vertex_ply(root/"little.ply")
        matrices=[np.eye(4,dtype=np.float32),np.array([[0,-1,0,2],[1,0,0,3],[0,0,1,1],[0,0,0,1]],np.float32)]
        xml="<MeshLabProject><MeshGroup>"+"".join('<MLMesh filename="little.ply"><MLMatrix44>'+" ".join(map(str,m.flat))+"</MLMatrix44></MLMesh>" for m in matrices)+"</MeshGroup></MeshLabProject>"
        (root/"scan_alignment.mlp").write_text(xml)
        scene=prepare_eth3d_remote_scene("electro",root,dataset_scale=1.7)
        chunks=list(iter_eth3d_remote_scene_chunks(scene))
        aligned=np.concatenate([c[0] for c in chunks]);aligned_colors=np.concatenate([c[1] for c in chunks])
        for bindex,binary in enumerate(a.binaries):
            for mode,source,points,expected_colors in [
                ("points",root/"little.ply",raw,colors),("points",root/"big.ply",raw,colors),
                ("cow",root/"little.ply",prepare_cow_lady_points(raw,world_scale=1.7),colors),
                ("eth3d",root,aligned,aligned_colors)]:
                lod=root/f"{bindex}-{mode}-{source.name}.hrl"
                result=subprocess.run([binary,mode,str(source),str(lod)],capture_output=True,text=True,timeout=30)
                assert result.returncode==0,result.stderr
                value=json.loads(result.stdout)
                np.testing.assert_allclose(value["points"],points,atol=5e-6,rtol=2e-6)
                np.testing.assert_array_equal(value["colors"],expected_colors)
                assert {s["scene_id"] for s in value["catalog"]["advanced"]}==set(ADVANCED_REMOTE_SCENES)
                for entry in value["catalog"]["advanced"]:
                    original=ADVANCED_REMOTE_SCENES[entry["scene_id"]]
                    assert entry["source_url"]==original.source_url and entry["license_name"]==original.license_name
                if mode=="eth3d":
                    for key in ("center","point_count","world_scale","source_bounds_min","source_bounds_max","depth_near_m","depth_far_m","initial_position","initial_pitch_degrees"):
                        np.testing.assert_allclose(value["scene"][key],getattr(scene,key),atol=5e-6,rtol=2e-6)
                    assert len(value["scene"]["fingerprint"])==64
                if mode!="cow":
                    raw_lod=lod.read_bytes();magic,version,voxel,count=struct.unpack("<4sIfQ",raw_lod[:20])
                    assert magic==b"HRL1" and version==1 and abs(voxel-.05)<1e-7
                    vertices=np.asarray(value["points"],np.float32)
                    unique={}
                    for xyz,rgb in zip(vertices,expected_colors):
                        key=tuple(np.floor(xyz*np.float32(1/np.float32(.05))).astype(np.int32))
                        unique.setdefault(key,struct.pack("<3f3B",*xyz,*rgb))
                    assert count==value["lod_count"]==len(unique)
                    assert raw_lod[20:]==b"".join(unique[k] for k in sorted(unique))
                    # Existing data must not be overwritten.
                    failed=subprocess.run([binary,mode,str(source),str(lod)],capture_output=True,timeout=30)
                    assert failed.returncode!=0 and lod.read_bytes()==raw_lod
                print(json.dumps(dict(binary=binary,mode=mode,source=source.name,points=len(points),result="passed")),flush=True)
            bad=root/"truncated.ply"
            bad.write_bytes((root/"little.ply").read_bytes()[:-3])
            result=subprocess.run([binary,"points",str(bad),str(root/"bad.hrl")],capture_output=True,timeout=10)
            assert result.returncode!=0 and b"Truncated" in result.stderr,result.stderr


if __name__=="__main__":main()
