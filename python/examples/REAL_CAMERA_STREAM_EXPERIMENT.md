# Real Camera Streaming Experiment

This experiment removes simulator rendering from the camera-rate comparison.
It replays a real TUM RGB-D recording on the robot-site PC, transports camera
video through either HORUS's native WebRTC source/packet relay or ROS
`sensor_msgs/CompressedImage` over Zenoh, and receives the other data through
the normal HORUS Connector. It sends no robot commands.

## Recording

- TUM RGB-D `freiburg1_desk`, J. Sturm et al., IROS 2012.
- [Dataset and attribution](https://cvg.cit.tum.de/data/datasets/rgbd-dataset).
- [Original file format](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats).
- CC BY 4.0. Preserve the dataset attribution when sharing results.
- Downloaded archive SHA-256:
  `e983d6830916e66dc4a46a71368046b149b283de87769690e7aa4e0b9483530c`.
- This archive contains 613 colour frames, 595 depth frames, 2,335 pose rows
  and 11,815 accelerometer rows. The colour timestamps span approximately
  20.4 seconds at 29.9936 Hz. Replay follows those timestamps at 1x speed.
- Three-camera trials are three phase-offset streams of this one recording,
  not three independently recorded robots. Each loop repeats the recording;
  there is no motion interpolation or frame duplication to claim higher FPS.

## Pipeline

```text
Arancino recorded RGB -> local ROS -> native GPU H.264 encoder
  -> WAN WebRTC over Tailscale -> native HORUS packet relay on this PC
  -> local WebRTC -> independent native GStreamer decoder -> Rerun display copy

Alternative: same recorded RGB -> live JPEG encoder -> ROS CompressedImage
  -> Zenoh -> local ROS -> independent JPEG decoder -> Rerun display copy

Arancino recorded pose / acceleration / depth / full coloured cloud
  -> local ROS -> HORUS Connector priority lanes -> local ROS -> observer
```

In WebRTC mode no JPEG camera stream crosses the WAN, and there is no
operator-side video decode/re-encode in the transport path. In ROS mode JPEG
frames cross the WAN and no WebRTC peer is created. The Rerun display adapter
is downstream of measurement; its JPEG95 display copies are not changes to
either transport's measured input.

The default observer uses the existing GStreamer/libnice environment, a 30 ms
jitter-buffer target, H.264 loss feedback and one decoding thread per camera.
Source identities are joined to decoded frames by presentation timestamp, not
FIFO order. `--webrtc-decoder aiortc` retains the earlier observer for diagnosis;
its receiver/decoder queues can add substantial delay, so label that backend
explicitly and do not interpret its output delay as network or Quest latency.
The native observer's packet tap is after its jitter/repair stage; its observed
RTP payload rate is not a wire bitrate or a raw network packet-loss measurement.
Unavailable native RTCP statistics must not be reported as zero loss.

## Current Installed Test

Deployment and results are under:

```text
/media/omotoye/New Volume/HORUS/diagnostics/real-camera-arancino-2026-11-01
```

Use the scripts in that folder:

```bash
cd "/media/omotoye/New Volume/HORUS/diagnostics/real-camera-arancino-2026-11-01"
bash start.sh
bash view.sh
```

To view the ROS compressed-image path instead:

```bash
bash view.sh --transport ros
```

Do not run a viewer alongside the serial benchmark.

`view.sh` opens the received streams, point cloud, measured trajectory, depth,
camera-rate graph and transport status in Rerun. Its default is ten minutes;
use `bash view.sh --duration 120` for a shorter session. It refuses a second
simultaneous receiver. It never starts Isaac Sim or drives a robot.

For serial measurements, with no live receiver running:

```bash
bash measure.sh
```

To stop only this deployment, retaining data and the Rerun window:

```bash
bash stop.sh
```

These launchers use the existing Arancino SSH alias and installed project-local
runtime, not a portable new installation. The previous hospital configs and
its 12 Mbit/s budget are unchanged. This experiment has a separate explicit
20 Mbit/s application budget, not a measured or guaranteed link capacity.

## Load And Timing

1. One colour camera, other sensor publication disabled.
2. One colour camera plus recorded pose, acceleration, depth and cloud.
3. Three colour streams plus the same sensor workload.

The transport comparison repeats one-camera and three-camera mixed-load
conditions serially in alternating order. Both use original 640x480 frames at
their native ~30 Hz cadence, without synthetic content. ROS uses live JPEG
quality 75; WebRTC requests 2500 kbit/s per H.264 stream and adapts to feedback.
These are **not equal-quality or equal-bitrate codecs**. Record actual bytes
and source-matched decoded image quality before interpreting bandwidth savings.
The earlier pre-encoded JPEG pilot excludes live JPEG encoding and must not be
used as encoding-CPU evidence.

The current machine's paired-run launcher is:

```bash
bash "/media/omotoye/New Volume/HORUS/diagnostics/stream-stability-2026-11-01/compare.sh" 45
```

It records two 45-second measurements per transport/stream-count condition,
with ten seconds of warmup per condition. It keeps failed trials, source
identities, payload hashes, optional decoded quality samples and host resource
traces. This is a diagnostic comparison, not a statistically powered experiment.
The source workers must acknowledge each trial ID before measurement. The
receiver returns the source to an idle condition on exit.

RGB retains all 640x480 pixels and original timing. Pose and acceleration use
their original recorded cadence. Mixed-load depth selects recorded frames at
approximately 2 Hz and encodes them as lossless PNG. The cloud selects frames
at approximately 0.2 Hz and includes **every valid depth pixel** and its colour;
this is not a fused map or the full 30 Hz depth workload. Publication/encoding
time can make these selected rates slightly lower. No artificial bulk bytes
or synthetic geometry are added.

Raw video, live JPEG, motion and bulk publishers run in independent processes, sharing a
scheduled replay epoch. This prevents Python serialization of a full cloud
from holding the camera publisher's interpreter. The local Fast DDS shared
memory segment must exceed the message/burst size: its default 512 KiB segment
is smaller than one 921,600-byte RGB image. This deployment uses a scoped
64 MiB segment, not changes to host network buffers or a quality reduction.

## Measurements

- The patched connector retains and tears down matching-listener handles with
  their routes. The reconnect regression checks that 30 publications remain
  30 deliveries across repeated publisher/subscriber recreation, not 60/90/120.
- This deployment opts into independent real-time, sensing and bulk Zenoh
  sessions. Their queues, DDS participants and reconnection lifetimes are
  separate. It uses TCP ports 7447--7449 and the same routing/priority policy
  at both ends. This does not create more network capacity or guarantee a
  latency bound on congested Wi-Fi. General installations remain opt-in.
- The installed experiment uses direction-specific routes: sensors flow from
  Arancino to the observer; only signaling, condition and clock requests return.
  This avoids feedback through broad bidirectional data-topic allowances.
  These profiles are scoped to this test, not replacements for general SDK
  registration, control or shared-workspace routes.
- Source CSV records actual publications, original dataset timestamp, loop,
  frame index, byte count and wall/monotonic time. A publication count alone
  is not proof of DDS/encoder receipt.
- The source encoder's SEI identity carries the original replay publication
  stamp through the packet relay and decoder. The analysis joins unique
  decoded identities to published frames, with two-second window-edge margins.
- Receiver CSV records decoded camera frames and sensor callbacks independently
  of Rerun. Duplicate sensor deliveries are reported separately; they must not
  inflate a stream's reported unique update rate.
- Results include source FPS, decoded FPS, frame-gap p95/p99/worst, unique identity
  delivery, source-to-decoder timing estimates and RTP receiver statistics.
- Count/full-window FPS and longest silence include leading/trailing outages;
  first-to-last frame cadence alone can hide a long freeze. Duplicate JPEG
  receipts are counted but cannot inflate unique decoded FPS. Depth, cloud and
  JPEG hashes are compared to source identities; validation is not inferred
  merely because an image decoder accepts its header.
- Timing uses repeated source clock request/reply, selecting recent low-RTT
  samples. The uncertainty bound is reported. This is **replay publication to
  PC decoder output**, not camera exposure, glass-to-glass, or Quest latency.
  The frame timestamp is assigned immediately before ROS message preparation
  (and before live JPEG encoding), so that local preparation is included.
- Resource traces report process CPU in percent of one CPU core, resident
  memory, whole-GPU utilization/memory and host Tailscale byte counters. Host
  network totals include unrelated traffic and are not per-stream wire rates.
- Rerun uses its own isolated environment and private display pipe. Its logged
  images are counted separately. Logged/display-copy rate is not physical
  monitor presentation FPS.

Keep failed, interrupted and overlapping setup trials, but do not combine
them with serial measurement conditions or hide poor network results. These
short diagnostic trials are not statistically replicated paper results.

## Tools

`tools/real_camera_source.py` is the source replay; `real_camera_viewer.py`
is the decoder/observer; `real_camera_rerun.py` is the separate display sink.
`real_camera_data.py` holds timing/cloud helpers, and
`analyze_real_camera_experiment.py` joins saved source/receiver evidence.
`real_camera_resources.py` is a read-only process/GPU/network sampler.
`real_camera_gstreamer.py` implements the native diagnostic receiver;
`real_camera_quality.py` compares captured frames to their exact dataset source;
`summarize_real_camera_comparison.py` emits the paired comparison and caveats.

No new system packages are required by the current deployment. Rerun 0.34.1
uses NumPy2/Arrow26 while the existing ROS/OpenCV decoder uses NumPy1; keep
these processes/environments separate rather than replacing shared dependencies.
