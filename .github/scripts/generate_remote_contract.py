#!/usr/bin/env python3
"""Generate deterministic native v3 conformance data using the Python SDK."""
from dataclasses import replace
import json
from pathlib import Path

from horus.remote_rendering.v3_protocol import (
    ClockAlignment, ClockSample, EyeView, Quaternion, RenderedFrameMetadata,
    Vector3, ViewState, WorkspaceState, eye_fov_from_unity_projection,
    pack_ros_diagnostic_frame,
)
from horus.remote_rendering.v3_renderer import (
    eye_projection_matrix, eye_view_matrix, eye_view_projection_matrix,
)


def main():
    q = Quaternion.from_value([0.13, 0.34, -0.17, 0.91])
    left = EyeView(Vector3(-0.032, 1.7, 0.1), q,
                   eye_fov_from_unity_projection(1.31, 1.47, 0.08, -0.04))
    right = replace(left, position=Vector3(0.032, 1.7, 0.1),
                    fov=eye_fov_from_unity_projection(1.29, 1.46, -0.07, -0.03))
    view = ViewState("operator-a", 17, 1000000, 1011111, 13888889, "workspace-local",
                     WorkspaceState(4, Vector3(1, 2, 3), Quaternion(0, 0, 0, 1), .1),
                     left, right, 1832, 1920, 72., "stream-a")
    metadata = RenderedFrameMetadata(
        "operator-a", 7, 17, 1011111, 4, 10, 20, 21, 22, "side_by_side", 4, 2, "png",
        view_stream_id="stream-a")
    # The carrier treats PNG bytes as opaque; image decoding belongs to the renderer.
    image = bytes.fromhex("89504e470d0a1a0a0102030405")
    clocks = [[1000, 1100, 1110, 1040], [2000, 2110, 2120, 2080],
              [3000, 3090, 3100, 3020], [4000, 4130, 4140, 4100],
              [5000, 5100, 5110, 5040], [6000, 6090, 6100, 6020]]
    alignment = ClockAlignment()
    for values in clocks:
        alignment.add(ClockSample(*values))
    matrices = {}
    for name, eye in (("left", left), ("right", right)):
        matrices[name + "_view"] = eye_view_matrix(eye).tolist()
        matrices[name + "_projection"] = eye_projection_matrix(eye, .1, 40.).tolist()
        matrices[name + "_vp"] = eye_view_projection_matrix(eye, .1, 40.).tolist()
    fixture = {"view": view.to_dict(), "metadata": metadata.to_dict(), "matrices": matrices,
               "image_hex": image.hex(), "wire_hex": pack_ros_diagnostic_frame(metadata, image).hex(),
               "clock_samples": clocks, "offset_ns": alignment.offset_ns,
               "round_trip_ns": alignment.round_trip_ns}
    destination = Path(__file__).resolve().parents[2] / "contracts/fixtures/remote_protocol.json"
    destination.write_text(json.dumps(fixture, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
