#!/usr/bin/env python3
"""
Unity Connection Test

This script demonstrates testing the Unity connection detection.
It can be used to simulate a Unity app connecting during SDK initialization.
"""

import os
import subprocess
import sys
import time


def main():
    print("🧪 HORUS Unity Connection Test")
    print("=" * 40)
    print("This test demonstrates Unity MR connection detection")
    print("")

    choice = input(
        "Choose test mode:\n1. Simulate Unity connection (run this first)\n2. Test SDK with Unity connected\n3. Show usage instructions\n\nEnter choice (1-3): "
    ).strip()

    if choice == "1":
        print("\n🎮 Starting Unity Connection Simulator...")
        print("Run this command in a separate terminal:")
        print(f"  python3 {os.path.abspath('examples/unity_connection_simulator.py')}")
        print("\nOr run it here (Ctrl+C to stop):")

        try:
            subprocess.run([sys.executable, "examples/unity_connection_simulator.py"])
        except KeyboardInterrupt:
            print("\n✅ Unity simulator stopped")

    elif choice == "2":
        print("\n🚀 Testing SDK with Unity connection...")
        print("Make sure Unity simulator is running in another terminal!")
        time.sleep(2)

        # Source ROS2 workspace and run SDK test
        sdk_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        workspace_setup = os.path.join(
            sdk_root, "horus_ros2_ws", "install", "setup.bash"
        )

        if os.path.exists(workspace_setup):
            cmd = f"source {workspace_setup} && python3 examples/quick_test.py"
            subprocess.run(["bash", "-c", cmd], cwd=sdk_root)
        else:
            print("❌ ROS2 workspace not built. Please run:")
            print("   cd horus_ros2_ws && colcon build")

    elif choice == "3":
        print("\n📖 Usage Instructions:")
        print("=" * 50)
        print("To test Unity connection detection:")
        print("")
        print("Terminal 1 (Unity Simulator):")
        print("  python3 examples/unity_connection_simulator.py")
        print("")
        print("Terminal 2 (SDK Test):")
        print("  source horus_ros2_ws/install/setup.bash")
        print("  python3 examples/quick_test.py")
        print("")
        print("Expected behavior:")
        print("• SDK will show 'Connected successfully' instead of 'Standby mode'")
        print("• Unity simulator will show 'Unity connection established'")
        print("• Both will display connection status with proper color coding")
        print("")
        print("To stop:")
        print("• Ctrl+C in Unity simulator terminal")
        print("• SDK test completes automatically")

    else:
        print("❌ Invalid choice")


if __name__ == "__main__":
    main()
