#!/usr/bin/env python3
"""
Quick HORUS SDK Test

This script tests the SDK from the current directory structure,
useful when the SDK is not installed system-wide.
"""

import os
import sys
from pathlib import Path

# Add SDK to path if not installed
sdk_root = Path(__file__).parent.parent
python_path = sdk_root / "python"
if python_path.exists():
    sys.path.insert(0, str(python_path))


def main():
    """Quick SDK test"""
    print("🚀 HORUS SDK Quick Test")
    print("=" * 30)

    try:
        from horus import Client

        print("✓ SDK imported successfully")

        print("\nInitializing with ROS2 backend...")
        client = Client(backend="ros2", auto_launch=True)
        print("✅ SDK test completed successfully!")

    except ImportError as e:
        print(f"❌ Import Error: {e}")
        print("\nMake sure you're running from the SDK directory or have it installed")
        return 1

    except Exception as e:
        print(f"❌ Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
