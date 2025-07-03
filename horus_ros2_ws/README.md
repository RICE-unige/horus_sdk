# HORUS ROS2 Backend

This workspace contains the HORUS ROS2 backend system with Unity integration support.

## Packages

### Core Packages
- **`horus_interfaces`**: Custom ROS2 message and service definitions
- **`horus_backend`**: Main C++ backend node for robot management

### Dependencies
- **`ros_tcp_endpoint`**: Unity TCP communication bridge (git submodule)

## Architecture

```
HORUS SDK (Python/C++)
    ↓ TCP (port 8080)
HORUS Backend Node (C++)
    ↓ ROS2 Topics/Services
ROS-TCP-Endpoint (Python)
    ↓ TCP (port 10000)
Unity Mixed Reality App
    ↓ Commands
Robot Systems
```

## Building

```bash
# Clone with submodules
git submodule update --init --recursive

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Launch Complete Backend (Recommended)
```bash
# Launches both HORUS backend and Unity TCP bridge
ros2 launch horus_backend horus_complete_backend.launch.py
```

### Launch Individual Components
```bash
# HORUS backend only
ros2 launch horus_backend horus_backend.launch.py

# Unity TCP bridge only
ros2 launch ros_tcp_endpoint endpoint.py
```

### Configuration Options
```bash
# Custom ports
ros2 launch horus_backend horus_complete_backend.launch.py tcp_port:=8080 unity_tcp_port:=10000

# Disable Unity bridge
ros2 launch horus_backend horus_complete_backend.launch.py enable_unity_bridge:=false

# Verbose logging
ros2 launch horus_backend horus_complete_backend.launch.py log_level:=debug
```

## Network Ports

- **8080**: HORUS SDK ↔ Backend communication
- **10000**: Unity ↔ ROS TCP Endpoint communication

## Integration with HORUS SDK

The HORUS SDK automatically launches this backend when initialized:

```python
from horus import Client
client = Client(backend='ros2')  # Automatically starts backend
```

## Development

### Adding Custom Messages
1. Add message definitions to `horus_interfaces/msg/`
2. Update `horus_interfaces/CMakeLists.txt`
3. Rebuild: `colcon build --packages-select horus_interfaces`

### Extending Backend Functionality
1. Modify `horus_backend/src/backend_node.cpp`
2. Add new services/topics in `horus_backend/include/horus_backend/backend_node.hpp`
3. Rebuild: `colcon build --packages-select horus_backend`

## Troubleshooting

### Port Conflicts
```bash
# Check if ports are in use
netstat -tulpn | grep 8080
netstat -tulpn | grep 10000

# Kill existing processes
pkill -f horus_backend_node
pkill -f default_server_endpoint
```

### Submodule Issues
```bash
# Update submodules
git submodule update --remote

# Reinitialize if needed
git submodule deinit -f .
git submodule update --init --recursive
```