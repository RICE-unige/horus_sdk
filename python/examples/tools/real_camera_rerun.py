"""Native Rerun display sink for the private local experiment pipe, not a transport."""

import argparse
from collections import deque
import pickle
from pathlib import Path
import sys
import uuid

import rerun as rr
import rerun.blueprint as rb


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--port", type=int, default=9891)
    parser.add_argument("--cameras", type=int, default=3)
    parser.add_argument("--transport", choices=("webrtc", "ros"), default="webrtc")
    args = parser.parse_args()
    rr.init("HORUS | Arancino Real Camera Experiment", recording_id=uuid.uuid4().hex)
    rr.spawn(port=args.port, memory_limit="1GiB", server_memory_limit="128MiB",
             hide_welcome_screen=True, connect=False,
             executable_path=str(Path(rr.__file__).parents[1] / "rerun_cli/rerun"))
    rr.set_sinks(rr.GrpcSink(f"rerun+http://127.0.0.1:{args.port}/proxy"),
                 rr.FileSink(args.output+"/received.rrd"))
    label = "WebRTC" if args.transport == "webrtc" else "ROS JPEG / Zenoh"
    cameras = [rb.Spatial2DView(origin=f"video/cam{i}", name=f"Received RGB {i} / {label}")
               for i in range(1, args.cameras+1)]
    rr.send_blueprint(rb.Blueprint(rb.Vertical(rb.Horizontal(*cameras),
        rb.Horizontal(rb.Spatial3DView(origin="world", name="Received Geometry & Motion"),
            rb.Spatial2DView(origin="depth", name="Received Depth / Metres")),
        rb.Horizontal(rb.TimeSeriesView(origin="rates", contents=["rates/decoded/**", "rates/source/**"],
                axis_y=rb.ScalarAxis(range=[0., 40.]), name="Source vs Decoded Camera FPS"),
            rb.TextDocumentView(origin="status", name="Live Transport Status")),
        row_shares=[.42, .36, .22]), collapse_panels=True))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/camera", rr.TransformAxes3D(.15), static=True)
    palette = [[29,190,180], [255,172,55], [130,140,255]]
    for i in range(1, args.cameras+1):
        rr.log(f"rates/decoded/cam{i}", rr.SeriesLines(colors=[palette[i-1]], names=[f"Decoded camera {i}"]), static=True)
    rr.log("world/camera/pinhole", rr.Pinhole(focal_length=[517.3,516.5],
        principal_point=[318.6,255.3], resolution=[640,480], image_plane_distance=.15), static=True)
    path = deque(maxlen=3000)
    try:
        while True:
            kind, stamp, data = pickle.load(sys.stdin.buffer)
            rr.set_time("receipt", duration=stamp)
            if kind == "image":
                rr.log(f"video/{data['name']}/rgb", rr.EncodedImage(contents=data["jpeg"], media_type="image/jpeg"))
            elif kind == "pose":
                path.append(data["position"])
                rr.log("world/camera", rr.Transform3D(translation=data["position"],
                    quaternion=data["quaternion"]))
                if len(path) % 10 == 0:
                    rr.log("world/trajectory", rr.LineStrips3D([list(path)], colors=[[255,172,55]], radii=.004))
            elif kind == "depth":
                rr.log("depth/image", rr.DepthImage(data, meter=5000.))
            elif kind == "cloud":
                if data["position"] is not None:
                    rr.log("world/cloud", rr.Transform3D(translation=data["position"], quaternion=data["quaternion"]))
                rr.log("world/cloud/points", rr.Points3D(data["points"], colors=data["colors"], radii=.004))
            elif kind == "status":
                for name, number in data["rates"].items():
                    rr.log("rates/"+name, rr.Scalars(number))
                rr.log("status", rr.TextDocument(data["text"], media_type="text/markdown"))
    except EOFError:
        pass
    finally:
        rr.get_global_data_recording().flush()
        rr.disconnect()


if __name__ == "__main__":
    main()
