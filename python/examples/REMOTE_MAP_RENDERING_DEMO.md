# 3D Map Rendering

HORUS exposes two independent SDK choices for 3D maps:

- `render_target="quest"` sends bounded geometry to the headset renderer.
- `render_target="remote"` keeps dense geometry on the PC GPU and sends RGB-D.
- `update_mode="static"` keeps the first complete remote atlas or map update.
- `update_mode="refresh"` renders from the latest Quest pose continuously.

The dense examples default to remote rendering. The PC loads the complete
source map into the NVIDIA GPU, renders RGB-D, and streams it through the HORUS
ROS2 WebRTC path. The Quest only reconstructs the streamed view. Unity is used
to build the APK and is not part of the runtime command chain.

## SDK API

```python
from horus.dataviz import MapRenderTarget, MapUpdateMode

# Dense map: rendering stays on the PC.
dataviz.add_3d_map(
    topic="/horus/remote_render/map_portal",
    render_target=MapRenderTarget.REMOTE,
    update_mode=MapUpdateMode.REFRESH,
    render_options={"transport": "webrtc", "framerate": 60},
)

# Bounded map: geometry is rendered directly on Quest.
dataviz.add_3d_map(
    topic="/bounded_map_3d",
    render_target=MapRenderTarget.QUEST,
    update_mode=MapUpdateMode.STATIC,
)
```

Do not select Quest rendering for the full Cow and Lady or ETH3D datasets.
That mode remains available for bounded maps whose geometry fits the headset
rendering and memory budget.

## Cow and Lady

Download the official ETH Zurich Cow and Lady fixture once:

```bash
cd ~/horus_sdk
python3 python/examples/tools/fetch_voxblox_cow_lady.py
```

Run the stable refresh mode:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH="$PWD/python:$PYTHONPATH"
python3 python/examples/remote_dense_map_portal_registration.py \
  --transport webrtc \
  --update-mode refresh \
  --stream-profile balanced \
  --workspace-scale 0.1
```

Run the stable fixed atlas:

```bash
python3 python/examples/remote_dense_map_portal_registration.py \
  --transport webrtc \
  --update-mode static \
  --stream-profile balanced \
  --workspace-scale 0.1
```

## ETH3D Courtyard

The Courtyard example loads two aligned colored laser scans containing
37,795,990 points into the PC renderer:

```bash
cd ~/horus_sdk
python3 python/examples/tools/fetch_eth3d_courtyard.py

source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH="$PWD/python:$PYTHONPATH"
python3 python/examples/remote_large_map_portal_registration.py \
  --transport webrtc \
  --update-mode refresh \
  --stream-profile balanced \
  --workspace-scale 0.1
```

The Courtyard renderer contains substantially more source points. Start with
`balanced`; it keeps the Quest decode and reprojection load bounded while the
PC publishes the newest completed render without building a backlog. Use
`fast60` only after the stable profile passes on the target headset and network.

Static Courtyard mode uses the complete fixed atlas:

```bash
python3 python/examples/remote_large_map_portal_registration.py \
  --transport webrtc \
  --update-mode static \
  --stream-profile balanced \
  --workspace-scale 0.1
```

## WebRTC Profiles

| Profile | Refresh view | Refresh FPS | Static atlas | Purpose |
|---|---:|---:|---:|---|
| `balanced` | 960x540 | 30 | 1440x810 | Default stable profile |
| `fast60` | 960x540 | 60 | 1440x810 | Opt-in low-latency test |
| `quality` | 1280x720 | 30 | 1920x1080 | Opt-in quality test |
| `quality60` | 1280x720 | 60 | 1920x1080 | Opt-in stress profile |

Static mode sends the immutable frame until the Quest captures it, then the
Quest closes that stream and keeps its local RGB-D texture. Explicit
`--render-width`, `--render-height`, `--render-fps`, and
`--render-bitrate-kbps` values override the selected profile.

For transport diagnostics only, replace `--transport webrtc` with
`--transport ros`. ROS compressed mode is capped at 15 FPS and is not the
performance path.

Runtime health is available from:

```bash
ros2 topic echo --once /horus/remote_render/agent_status
ros2 topic hz /horus/remote_render/viewer_pose
```

The status payload reports source publication FPS, render-update target,
actual CUDA render time, pose age, source point count, and valid depth
fraction. The WebRTC stream rate and unique PC render rate are separate by
design: when rendering is slower than encoding, only the newest completed
frame is sent and no stale-frame queue is allowed to grow.

Sources:

- Cow and Lady: <https://doi.org/10.3929/ethz-b-000721636>
- ETH3D datasets: <https://www.eth3d.net/datasets>
