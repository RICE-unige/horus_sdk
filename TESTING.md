# HORUS SDK Testing Guide

This document provides information about testing the HORUS SDK.

## Quick Testing

### Recommended Test Command
```bash
cd /path/to/horus_sdk
python3 extra/dev_test.py
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
   - Complete development SDK initialization
   - Animated spinners and color-coded output
   - Automatic backend cleanup
   - Full connection testing

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
  ✓ Backend startup: Ready on port 8080
  ✓ Backend connection: Connected on port 8080

SDK initialized successfully
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

#### "Port 8080 is in use"
```bash
# Kill existing backend processes
pkill -f horus_backend_node
```

#### Import errors
The test files automatically handle PYTHONPATH setup, but if running manually:
```bash
export PYTHONPATH="/path/to/horus_sdk/python:$PYTHONPATH"
```

### Debug Mode
For verbose output, check the individual test files and uncomment debug print statements or modify log levels in the backend configuration.

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