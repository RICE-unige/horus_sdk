# HORUS SDK Testing Guide

This document provides information about testing the HORUS SDK.

## Quick Testing

### Recommended Test Command
```bash
cd /path/to/horus_sdk
python3 extra/dev_test.py
# Runs continuous MR app monitoring - Press Ctrl+C to stop
```

### Alternative Test Commands
```bash
# Clean test with backend cleanup
python3 extra/clean_test.py

# Import and initialization test  
python3 extra/test_import.py

# Basic SDK initialization test
python3 extra/test_sdk_init.py

# Interactive shell setup
./extra/horus_shell.sh
```

## Test Structure

All test files are located in the `extra/` directory which is git-ignored for development convenience.

### Available Tests

1. **`dev_test.py`** *(Recommended)*
   - Continuous HORUS MR app connection monitoring
   - Real-time Quest 3 device connection detection
   - Animated spinners and color-coded output
   - Clean shutdown with Ctrl+C
   - Comprehensive backend and process cleanup

2. **`clean_test.py`**
   - Clean test environment setup
   - Kills existing backend processes
   - Basic initialization testing

3. **`test_import.py`**
   - Import functionality testing
   - Environment setup verification
   - SDK module availability check

4. **`test_sdk_init.py`**
   - Basic SDK initialization
   - Simple pass/fail testing
   - Minimal output

5. **`horus_shell.sh`**
   - Interactive shell with HORUS environment
   - Pre-configured PYTHONPATH and ROS2 workspace
   - Development convenience script

## Expected Test Output

### Successful Development Test
```
Initializing ROS2 backend
  ✓ ROS2 Installation: ROS2 humble detected
  ✓ HORUS Backend Package: HORUS backend package found  
  ✓ Network Port 8080: Port 8080 is available
  ✓ Unity TCP Endpoint (Port 10000): Running on port 10000
  ✓ Backend startup: Ready on port 8080
  ✓ Backend connection: Connected on port 8080
  ✓ TCP endpoint: Running on port 10000

🎮 HORUS Mixed Reality Connection
═════════════════════════════════════════════
  📡 Connection Details:
     IP Address: 10.104.127.215
     Port:       10000
     Protocol:   TCP

  🔗 HORUS MR App Configuration:
     Enter these details in your Quest 3 HORUS app
     Host: 10.104.127.215:10000

  ⏳ Monitoring for HORUS MR connections...
  ⧖ MR connection: Standby mode (monitoring for connections)
    → Launch HORUS app on Quest 3 to connect

SDK initialized successfully

HORUS MR Connection Monitoring Active
📝 Connect your HORUS MR app to see real-time connection detection!
   Connection events will appear above when Quest 3 connects/disconnects
   Press Ctrl+C to stop monitoring and shutdown cleanly

⏳ Monitoring connections... (Press Ctrl+C to stop)

# When Quest 3 connects:
✓ HORUS MR connection: Connected from 10.104.127.201
  → Mixed Reality interface active

# When Quest 3 disconnects:
⧖ HORUS MR connection: Disconnected from 10.104.127.201
  → Monitoring for new connections...
```

## Prerequisites

### Required Components
- ROS2 Humble installation
- HORUS workspace built (`colcon build`)
- Python 3.8+ with required packages

### Build Commands
```bash
# Build the workspace
cd horus_ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```

## Troubleshooting

### Common Issues

#### "Missing requirements: HORUS Backend Package"
```bash
# Ensure workspace is sourced
cd horus_ros2_ws
source install/setup.bash
```

#### "Port 8080 is in use" or "Port 10000 is in use"
```bash
# Kill existing backend processes (comprehensive cleanup)
pkill -f horus_backend_node
pkill -f default_server_endpoint
pkill -f ros_tcp_endpoint

# Or use the cleanup test
python3 extra/test_cleanup.py
```

#### Import errors
The test files automatically handle PYTHONPATH setup, but if running manually:
```bash
export PYTHONPATH="/path/to/horus_sdk/python:$PYTHONPATH"
```

### Debug Mode
For verbose output, check the individual test files and uncomment debug print statements or modify log levels in the backend configuration.

### HORUS MR App Connection Testing
To test with a real Quest 3 device:
1. Run `python3 extra/dev_test.py`
2. Note the IP address and port displayed (e.g., 10.104.127.215:10000)
3. Enter these details in your Quest 3 HORUS app
4. Watch for real-time connection events in the SDK output
5. Test connecting/disconnecting to verify monitoring works
6. Press Ctrl+C to cleanly shutdown all processes

## Development Testing

### Adding New Tests
1. Create test files in `extra/` directory
2. Use relative paths for cross-platform compatibility
3. Follow the existing pattern for environment setup
4. Test files should be self-contained and not require external arguments

### Test File Template
```python
#!/usr/bin/env python3
import sys
import os

# Get SDK root directory
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(script_dir, 'python'))

# Your test code here
from horus import Client
client = Client(backend='ros2')
```
