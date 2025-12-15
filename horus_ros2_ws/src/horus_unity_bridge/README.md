# HORUS Unity Bridge

High-performance C++ ROS 2 node inspired by the Unity ROS‑TCP‑Endpoint and built as its drop-in replacement for the HORUS mixed-reality stack.  
One process hosts the epoll-based TCP bridge, topic router, and service manager, delivering sub-millisecond routing while preserving the original Unity protocol semantics.

## Quick Start

```bash
# Build inside the HORUS workspace
cd ~/horus_sdk/horus_ros2_ws
colcon build --packages-select horus_unity_bridge
source install/setup.bash

# Run the bridge (listens on tcp/10000 by default)
ros2 run horus_unity_bridge horus_unity_bridge_node
```

Supply parameters through `config/bridge_config.yaml` or `--ros-args`.  
Important knobs: `tcp_ip`, `tcp_port`, socket buffer sizes, queue depth, worker thread count, and default QoS profiles.

## Using the Bridge

Unity continues to use the standard ROS‑TCP‑Connector API:

```csharp
ROSConnection.GetOrCreateInstance().Connect("bridge-ip", 10000);
ROSConnection.GetOrCreateInstance().Subscribe<PoseMsg>("/robot/pose", callback);
```

System commands (`__subscribe`, `__publish`, `__ros_service`, etc.), binary framing, and keep‑alives all match the upstream protocol.  
Topic data is published via ROS generic publishers/subscriptions and services are proxied through the typed `ServiceManager`.

## Monitoring & Operations

- Structured logs list connections, per‑topic registrations, and every 30 s statistics covering TCP traffic, routed messages, and service counts.
- To change runtime settings, edit `config/bridge_config.yaml` and relaunch (hot reload is not yet supported).
- If the port is busy, free it via `fuser -k 10000/tcp`.

## WebRTC Video Streaming (Experimental)

### Prerequisites

WebRTC support requires GStreamer, OpenCV, and their development packages. Install them:

```bash
sudo apt-get update
sudo apt-get install -y \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  gstreamer1.0-x \
  gstreamer1.0-alsa \
  gstreamer1.0-gl \
  gstreamer1.0-gtk3 \
  gstreamer1.0-pulseaudio \
  libopencv-dev
```

### Build Configuration

- When GStreamer and OpenCV are present, CMake automatically fetches [`libdatachannel`](https://github.com/paullouisageneau/libdatachannel) and enables the `WebRTCManager`.
- The bridge can then accept Unity WebRTC offers and stream both raw (`sensor_msgs/msg/Image`) and compressed (`sensor_msgs/msg/CompressedImage`) topics through a low-latency pipeline.
- Use `-DENABLE_WEBRTC=OFF` to keep the feature disabled even if dependencies are installed.

### Runtime Parameters

Expose them via `config/bridge_config.yaml` or `--ros-args`:

| Parameter | Description | Default |
| --------- | ----------- | ------- |
| `webrtc.enabled` | Enable WebRTC signaling + streaming | `false` |
| `webrtc.camera_topic` | Image topic to publish to the peer connection | `/camera/image_raw` |
| `webrtc.bitrate_kbps` | Target encoder bitrate | `2000` |
| `webrtc.framerate` | Target framerate for caps + keyframe interval | `30` |
| `webrtc.encoder` | GStreamer encoder element (e.g., `x264enc`, `nvh264enc`) | `x264enc` |
| `webrtc.pipeline` | Optional custom pipeline string (overrides auto-generated) | `""` |
| `webrtc.stun_servers` | Array of STUN/ICE servers | `[stun:stun.l.google.com:19302]` |

### Image Format Support

**Raw Images** (`sensor_msgs/msg/Image`):
- `rgb8`, `bgr8`, `mono8` - zero-copy or efficient conversion
- Unsupported encodings logged once with warning

**Compressed Images** (`sensor_msgs/msg/CompressedImage`):
- Automatic detection based on topic name (e.g., `/camera/image/compressed`)
- JPEG and PNG formats supported via OpenCV decompression
- Automatically converted to RGB for WebRTC pipeline

**Topic Auto-Detection**:
- Topics containing `/compressed` or `_compressed` → CompressedImage subscription
- All other topics → raw Image subscription
- Or explicitly set the correct topic in `webrtc.camera_topic`

### Notes

- The default pipeline generates RTP/H.264 output. Provide your own via `webrtc.pipeline` if you need HEVC or hardware-specific flags.
- SDP/mLineIndex handling currently defaults to 0 as Unity can map it from `sdpMid`; extend as needed for multi-track sessions.

## Testing

Build the optional `horus_unity_bridge_test` package to access ROS publishers, service servers, Unity client simulators, and WebRTC offer/candidate smoke tests:

```bash
colcon build --packages-select horus_unity_bridge horus_unity_bridge_test
source install/setup.bash

# Example
ros2 run horus_unity_bridge horus_unity_bridge_node &
ros2 run horus_unity_bridge_test test_publisher
ros2 run horus_unity_bridge_test unity_client_simulator
```

Use `colcon test --packages-select horus_unity_bridge` to exercise C++ unit/integration tests.

## License

Apache-2.0 (same as the rest of HORUS SDK).
