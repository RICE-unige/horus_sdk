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

## Testing

Build the optional `horus_unity_bridge_test` package to access ROS publishers, service servers, and Unity client simulators:

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
