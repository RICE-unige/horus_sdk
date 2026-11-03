# Teach Camera Portals

Teach can display a camera as a binocular window with softly faded, rounded
edges. Each camera opts in independently. Wrist cameras can remain flat while
the front camera uses a portal. Existing registrations stay unchanged.

## Choose the Source

| Mode | Source | Depth information |
| --- | --- | --- |
| `mono` | One RGB stream | Flat window with softened edges; no invented stereo |
| `stereo` | Rectified left/right ROS images, or one side-by-side stream | Captured binocular disparity |
| `estimated` | Side-by-side preview generated on the host from one RGB stream | Estimated relative depth, explicitly labelled **not measured** |

The headset selects the appropriate image for each eye. It does not run depth
inference or copy the camera back from the GPU. These are camera windows, not
head-tracked 6-DoF reconstructions: moving your head cannot reveal surfaces that
the source cameras did not observe. This does not recreate Meta's proprietary
spatial-media processing.

## Real Stereo

Add the sensor and its visualization using your normal robot registration:

```python
from horus.sensors import Camera
from horus.teaching import TeachCamera, TeachPortal, TeachStream

front = Camera(
    "front", "front_left_optical", "/robot/front/left/image_rect",
    is_stereo=True, stereo_layout="dual_topic",
    right_topic="/robot/front/right/image_rect",
    resolution=(640, 480), encoding="rgb8",
    minimap_streaming_type="ros", teleop_streaming_type="ros",
    minimap_image_type="raw", teleop_image_type="raw",
)
robot.add_sensor(front)
view.add_sensor_visualization(front, robot.name)

front_view = TeachCamera(
    "front", stream_id="front_left", required_for_control=True,
    position=(0.0, 1.55, 1.1), width_m=0.8,
    portal=TeachPortal(mode="stereo", edge_fade=0.06, crop=0.02),
)
front_streams = (
    TeachStream("front_left", front.topic, "sensor_msgs/msg/Image", role="camera",
                source_identity="front-left", stamp_semantics="acquisition"),
    TeachStream("front_right", front.right_topic, "sensor_msgs/msg/Image", role="camera",
                source_identity="front-right", stamp_semantics="acquisition"),
)
```

Include `front_view` in your `TeachConfig.cameras` and `front_streams` in
`TeachConfig.streams`. Keep the robot-specific control adapter and recording
configuration unchanged. Record both original eyes and their calibration.

The inputs must be rectified, have matching resolution and use a common
timestamp clock. HORUS does **not** calibrate or rectify two arbitrary cameras.
By default, source timestamps must be within 50 ms; choose a tighter bound for
motion-sensitive work. Missing timestamps, an old eye, unequal dimensions or an
excessive timestamp difference produce a labelled **mono fallback**. A required
stereo portal is not counted as ready for control during this fallback.

For an already packed pair, use `stereo_layout="side_by_side"` and one topic,
left half first. The packed width must be even; the displayed aspect uses one
half, not the full texture. WebRTC can carry this packed format using HORUS's
existing camera transport. Separate-eye topics use ROS; HORUS does not assume
that independent WebRTC sessions are synchronized.

## Single RGB: Estimated Preview

The optional converter runs locally on the camera/ROS host. It uses the
Apache-2.0 **Depth Anything V2 Small** model at the pinned revision
`5426e4f0f36572d16453bbda7a8389317b1bef99`. No image is uploaded to a service.
Only the first explicit model download needs Internet access.

Install into a dedicated environment, not your robot's runtime environment:

```bash
cd ~/horus_sdk
python3 -m venv ~/.venvs/horus-portal
# CPU-only build; for CUDA choose the matching PyTorch build for your host.
~/.venvs/horus-portal/bin/pip install torch==2.6.0 --index-url https://download.pytorch.org/whl/cpu
~/.venvs/horus-portal/bin/pip install -e '.[camera-portal]'
source /opt/ros/jazzy/setup.bash
# Use the SAME ROS domain as your registration and camera source.
export ROS_DOMAIN_ID=58
export ROS_LOCALHOST_ONLY=1
PYTHONPATH=python:$PYTHONPATH ~/.venvs/horus-portal/bin/python -m horus.teaching.camera_portal \
  --input /robot/front/image_raw \
  --output /robot/front/portal_sbs/compressed \
  --model-cache ~/.cache/horus/portal-depth --download-model
```

Omit `--download-model` after preparation. Add `--compressed` for a compressed
input. `--device cuda` requires a CUDA-enabled PyTorch installation and spare
GPU memory. `--fps 10` is an output ceiling, **not** a guaranteed rate. The
converter reports actual published, replaced and age-dropped counts. Frames
older than `--max-age 0.5` seconds in the converter are dropped. This age budget
does not include transport or camera exposure latency.

Register the original image for the minimap and recordings, and the derived
topic only for the Teach display:

```python
front = Camera(
    "front", "front_optical", "/robot/front/image_raw",
    is_stereo=True, stereo_layout="side_by_side",
    minimap_topic="/robot/front/image_raw", minimap_image_type="raw",
    teleop_topic="/robot/front/portal_sbs/compressed", teleop_image_type="compressed",
    minimap_streaming_type="ros", teleop_streaming_type="ros",
)
front_view = TeachCamera(
    "front", stream_id="front_raw", required_for_control=True,
    portal=TeachPortal(mode="estimated"),
)
# TeachStream("front_raw", "/robot/front/image_raw", "sensor_msgs/msg/Image", ...)
```

Depth-dependent warping supplies binocular cues, not reliable physical
distances. Reflective surfaces, thin objects, occlusion boundaries and rapid
motion can be wrong. Newly exposed surfaces cannot be recovered. The default
per-eye displacement is limited to 0.8% of image width; `--strength` accepts
0 through 0.015. Do not use the output as ground-truth depth, a collision input,
or an original training observation. Keep raw images, source timestamps and
calibration in the dataset. Inferred depth is never sent to the robot adapter.

CPU conversion may be too slow for manipulation. A local check of twelve
1280x720 recorded G1 frames at inference longest-edge size 294 measured approximately 115 ms
median processing time on the development host, excluding network and display.
This is a small implementation check under concurrent development load, not a
platform benchmark or safety guarantee. Only the depth estimate uses the smaller
inference input; the RGB output retains its original per-eye resolution. Prefer real stereo for low-latency
operation; validate end-to-end latency before using any estimated view to act.

## Appearance

`TeachPortal` is supported in Python, C++ and Rust registrations with the same
normalized profile/hash. The preview converter is a Python host utility usable
with any of these registration clients.

| Field | Default | Meaning |
| --- | --- | --- |
| `mode` | `stereo` | `mono`, `stereo`, or `estimated` |
| `edge_fade` | `0.06` | Feather width as a fraction of the panel, 0-0.25 |
| `corner_radius` | `0.08` | Rounded corner fraction, 0-0.25 |
| `crop` | `0.02` | Inset within each eye image, 0-0.1 |
| `swap_eyes` | `false` | Correct a source whose left/right order is reversed |
| `max_skew_ms` | `50` | Maximum separate-eye timestamp difference, 0-100 ms |

Start with restrained disparity and a comfortable screen width/distance.
Cropping does not correct calibration errors. Stop viewing if the stereo is
uncomfortable; use `mode="mono"` to retain the portal edges without disparity.
Removing `portal` restores the ordinary Teach camera presentation.

## Hardware-Free Check

With an app build containing portal support:

```bash
cd ~/horus_sdk
./teach-demo --continuous --front-portal stereo
```

Open **Robot Manager > Teleop > Teach**. The front view contains three coloured
diagnostic targets at distinct depths, rendered with a known pinhole stereo
pair. Wrist views remain unchanged. This fixture tests presentation, not a real
robot camera or a training dataset. No depth model is needed.

For the optional conversion path, start the converter on domain 58 with
`--input /teach_synthetic/front/image_raw` and
`--output /teach_synthetic/front/portal_sbs/compressed`, then run
`./teach-demo --continuous --front-portal estimated`. The ordinary synthetic
RGB fixture is not a depth-quality benchmark; use recorded real RGB for quality
evaluation. Existing examples without `--front-portal` retain their old behavior.

## If the View Still Looks Flat

The SDK option alone cannot update the headset. Install an app build containing
portal support before running the demo; an older app can accept the registration
while ignoring the new presentation fields. Both stereo routing and the edge
fade will then be absent.

Open **Teleop > Teach**, not the robot's ordinary projected camera or minimap.
No engagement or recording is needed to view the cameras. The front camera label
should say **Stereo portal** or **Estimated depth | Not measured**. A **Mono
fallback** label means the stereo pair is unavailable or outside its freshness,
resolution or timestamp limits. Check both source topics before changing eye order.
The wrist cameras in this demo deliberately remain ordinary flat views.

`--front-portal stereo` uses a generated left/right test pair, not monocular
conversion. `--front-portal estimated` additionally requires the host converter
described above. Merely cropping the same image differently per eye cannot
recover depth variation within the scene.

### References

- [Depth Anything V2 model and licensing](https://github.com/DepthAnything/Depth-Anything-V2)
- [Pinned Small model](https://huggingface.co/depth-anything/Depth-Anything-V2-Small-hf/tree/5426e4f0f36572d16453bbda7a8389317b1bef99)
- [Unity single-pass instanced stereo shader requirements](https://docs.unity3d.com/6000.0/Documentation/Manual/SinglePassInstancing.html)
