#!/usr/bin/env python3
"""Native Marker/nvblox source -> TF -> HMSH checks on isolated localhost DDS.

Publishes test maps and static TF only; never publishes robot commands.
"""
import argparse
import copy
import json
import struct
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point, Point32, TransformStamped
from std_msgs.msg import String, UInt8MultiArray, ColorRGBA
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


def header(data):
    values = struct.unpack_from("<IIIiiiQQBBHII", data)
    assert values[:2] == (0x484D5348, 2)
    assert len(data) == 52 + values[-2]*16 + values[-1]*4
    return {"chunk": values[2], "epoch": values[6], "revision": values[7],
            "op": values[8], "vertices": values[-2], "indices": values[-1]}


def check(binary, kind, number):
    node = rclpy.create_node(f"mesh_source_peer_{number}")
    root = f"/native_mesh_source_{number}"
    seen, statuses, manifests = [], [], []
    qos = QoSProfile(depth=16, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    if kind == "Marker":
        qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.BEST_EFFORT)
    if kind == "nvblox":
        from nvblox_msgs.msg import Mesh, MeshBlock, Index3D
        msgtype = Mesh
    else:
        msgtype = Marker if kind == "Marker" else MarkerArray
    source = node.create_publisher(msgtype, root+"/source", qos)
    # Fast DDS does not permit two types for one topic in a single participant.
    # A separate context reproduces an unrelated process advertising a bad type.
    wrong_context = rclpy.context.Context()
    wrong_context.init()
    wrong_node = rclpy.create_node(f"wrong_map_type_{number}", context=wrong_context)
    wrong_type = wrong_node.create_publisher(String, root+"/source", 1)
    tf = node.create_publisher(TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    request = node.create_publisher(String, root+"/map/indexed_request", 16)
    node.create_subscription(UInt8MultiArray, root+"/map/indexed", lambda m: seen.append(bytes(m.data)), 64)
    node.create_subscription(String, root+"/map/status", lambda m: statuses.append(json.loads(m.data)), 8)
    node.create_subscription(String, root+"/map/indexed_manifest", lambda m: manifests.append(json.loads(m.data)), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    process = subprocess.Popen([binary,kind,root+"/source",root+"/map"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.01)

    def until(condition, seconds=4):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if condition():
                return
            if process.poll() is not None:
                raise AssertionError(process.communicate())
        raise AssertionError({"kind":kind,"pages":[header(v) for v in seen],"status":statuses[-1:]})

    def marker(index):
        m = Marker(ns="room", id=index, type=Marker.TRIANGLE_LIST, action=Marker.ADD)
        m.header.frame_id = root.lstrip("/")+"/sensor"
        m.pose.position.y = 2.
        m.pose.orientation.w = 1.
        m.scale.x, m.scale.y, m.scale.z = 2., 1., 1.
        m.color = ColorRGBA(r=1., a=1.)
        m.points = [Point(x=index*5.,y=0.,z=0.),Point(x=index*5.+1,y=0.,z=0.),Point(x=index*5.,y=1.,z=0.)]
        return m

    def send(markers):
        if kind == "Marker":
            for m in markers: source.publish(m)
        elif kind == "MarkerArray":
            source.publish(MarkerArray(markers=markers))
        else:
            m=Mesh()
            m.header.frame_id=markers[0].header.frame_id
            m.clear=any(item.action==Marker.DELETEALL for item in markers)
            for item in markers:
                if item.action==Marker.DELETEALL: continue
                m.block_indices.append(Index3D(x=item.id-1,y=0,z=0))
                block=MeshBlock()
                if item.action==Marker.ADD:
                    block.vertices=[Point32(x=p.x*2,y=p.y+2,z=p.z) for p in item.points]
                    block.triangles=[0,1,2]
                    block.colors=[item.color]*3
                m.blocks.append(block)
            source.publish(m)

    try:
        until(lambda:source.get_subscription_count()>0)
        until(lambda:statuses and statuses[-1]["source_connected"])
        assert statuses[-1]["source_publishers"] == 1, "Wrong-type publisher counted as a map source"
        initial=[marker(0),marker(1)]
        send(initial)
        # TF arrives late: the original update must be deferred rather than dropped.
        until(lambda: statuses and statuses[-1]["tf_wait_messages"]>0)
        assert not seen
        transform=TransformStamped()
        transform.header.frame_id="map"
        transform.child_frame_id=initial[0].header.frame_id
        transform.transform.translation.x=10.
        transform.transform.translation.z=1.
        transform.transform.rotation.w=1.
        tf.publish(TFMessage(transforms=[transform]))
        until(lambda: len([p for p in seen if header(p)["op"]==0])==2)
        originals=seen[:]
        positions={struct.unpack_from("<fff",p,52) for p in originals}
        assert positions=={(10.,2.,1.),(20.,2.,1.)}, positions
        for page in originals:
            assert header(page)["indices"]==3 and page[64:68]==bytes([255,0,0,255])
        send(initial)
        spin(.3)
        assert seen==originals, "Unchanged source resent map geometry"
        bad=copy.deepcopy(initial)
        bad[-1].points[0].x=float("nan")
        if kind=="Marker": bad=bad[-1:]
        send(bad)
        spin(.3)
        assert seen==originals, "Invalid update changed retained map"
        deleted=marker(0);deleted.action=Marker.DELETE
        send([deleted])
        until(lambda:any(header(p)["op"]==1 for p in seen))
        # Request a late snapshot after repair coalescing cooldown.
        spin(2.1)
        count=len(seen)
        request.publish(String(data='{"request":"snapshot"}'))
        until(lambda:any(header(p)["op"]==0 for p in seen[count:]))
        repaired=[p for p in seen[count:] if header(p)["op"]==0]
        assert len(repaired)==1 and struct.unpack_from("<fff",repaired[0],52)==(20.,2.,1.)
        clear=marker(0);clear.action=Marker.DELETEALL
        send([clear])
        until(lambda: manifests and manifests[-1]["pages"]==[] and any(header(p)["op"]==2 for p in seen))
        node.destroy_publisher(source)
        until(lambda:statuses and not statuses[-1]["source_connected"])
        assert statuses[-1]["source_publishers"] == 0, "Wrong-type publisher kept source status connected"
        while process.poll() is None: spin(.05)
        stdout,stderr=process.communicate(timeout=2)
        assert process.returncode==0,(stdout,stderr)
        final=json.loads(stdout.strip().splitlines()[-1])
        assert final["page_count"]==0 and final["rejected"]>=1 and final["tf_wait_messages"]==0,final
        assert final["unchanged_pages"]>=2,final
        print(json.dumps({"binary":binary,"kind":kind,"result":"passed","late_tf":True,"geometry_and_color":True,"repair":True}),flush=True)
    finally:
        if process.poll() is None:
            process.terminate()
            try: process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill();process.communicate()
        node.destroy_node()
        wrong_node.destroy_node()
        wrong_context.shutdown()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("executables",nargs="+")
    parser.add_argument("--include-nvblox",action="store_true")
    args=parser.parse_args()
    rclpy.init()
    try:
        for i,binary in enumerate(args.executables):
            for j,kind in enumerate(["Marker","MarkerArray"]+(["nvblox"] if args.include_nvblox else [])):
                check(binary,kind,i*3+j)
    finally:
        rclpy.shutdown()


if __name__=="__main__": main()
