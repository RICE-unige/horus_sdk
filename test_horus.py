#!/usr/bin/env python3
"""
Quick test launcher for HORUS SDK
"""
import os
import subprocess
import sys


def main():
    # Get the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    extra_dir = os.path.join(script_dir, "extra")

    # Check if extra directory exists
    if not os.path.exists(extra_dir):
        print("❌ Extra directory not found. Please ensure test files are in ./extra/")
        sys.exit(1)

    # Available tests
    tests = {
        "1": (
            "dev_test.py",
            "Continuous Unity monitoring (Ctrl+C to stop) - Recommended",
        ),
        "2": ("test_unity_connection.py", "Unity MR connection monitoring test"),
        "3": ("test_cleanup.py", "Test comprehensive process cleanup"),
        "4": ("clean_test.py", "Clean test with backend cleanup"),
        "5": ("test_import.py", "Import and initialization test"),
        "6": ("test_sdk_init.py", "Basic SDK initialization test"),
    }

    print("🚀 HORUS SDK Test Launcher")
    print("=" * 40)

    for key, (filename, description) in tests.items():
        print(f"{key}. {description}")

    print("\nEnter test number (1-6) or 'q' to quit:")

    choice = input("> ").strip()

    if choice.lower() == "q":
        print("Goodbye!")
        return

    if choice not in tests:
        print("❌ Invalid choice")
        return

    filename, description = tests[choice]
    test_path = os.path.join(extra_dir, filename)

    if not os.path.exists(test_path):
        print(f"❌ Test file {filename} not found in extra/")
        return

    print(f"\n🔧 Running: {description}")
    print("=" * 60)

    # Run the test
    try:
        subprocess.run([sys.executable, test_path], cwd=script_dir, check=True)
    except subprocess.CalledProcessError as e:
        print(f"❌ Test failed with exit code {e.returncode}")
    except KeyboardInterrupt:
        print("\n⚠️ Test interrupted by user")


if __name__ == "__main__":
    main()
