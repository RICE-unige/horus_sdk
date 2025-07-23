# Client API

# Client API

## Overview

The `Client` class is the main entry point for the HORUS SDK. It manages backend connections and orchestrates communication between your robots and the Mixed Reality system.

## Usage

```python
from horus import Client

# Initialize with ROS2 backend
client = Client(backend='ros2')

# Use the client to manage robot registrations
# (Robot registration is handled through Robot.register_with_horus())
```

## Coming Soon

Complete API documentation will be automatically generated from the Python source code docstrings.
