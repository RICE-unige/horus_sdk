#!/usr/bin/env python3
"""
Basic HORUS SDK Usage Example

This example demonstrates the simplest way to initialize the HORUS SDK
and start all backend services for robot communication.

Prerequisites:
- Install the SDK: pip install -e /path/to/horus_sdk/python
- Or run from a virtual environment with the SDK installed
"""

import time

def main():
    """Basic SDK initialization example"""
    print("🚀 HORUS SDK Basic Initialization Example")
    print("=" * 50)
    
    try:
        # Import and initialize the HORUS SDK
        from horus import Client
        
        print("\n1. Importing HORUS SDK...")
        print("   ✓ SDK imported successfully")
        
        print("\n2. Initializing SDK with ROS2 backend...")
        # This will:
        # - Show ASCII art branding
        # - Check ROS2 requirements
        # - Launch the backend automatically
        # - Establish connections
        client = Client(backend='ros2', auto_launch=True)
        
        print("\n3. SDK initialized successfully!")
        print("   ✓ Backend services running")
        print("   ✓ Ready for robot communication")
        
        # Keep the connection alive for demonstration
        print("\n4. Keeping connection alive for 10 seconds...")
        print("   (Backend will continue running in background)")
        
        for i in range(10, 0, -1):
            print(f"   {i}...", end=" ", flush=True)
            time.sleep(1)
        
        print("\n\n✅ Example completed successfully!")
        print("   Backend services remain active for other applications")
        
    except ImportError as e:
        print(f"❌ Import Error: {e}")
        print("\n   Install the SDK first:")
        print("   \u2022 With sudo: sudo pip install -e python/")
        print("   \u2022 Virtual env: python3 -m venv env && source env/bin/activate && pip install -e python/")
        print("\n   Or use the quick_test.py (no installation needed):")
        print("   source horus_ros2_ws/install/setup.bash")
        print("   python3 examples/quick_test.py")
        return 1
        
    except Exception as e:
        print(f"❌ Initialization Error: {e}")
        print("   Check that ROS2 is installed and workspace is built")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())