#!/usr/bin/env python3
"""Verify frames emitted by each native implementation with the Python reader."""
import argparse
import json
from pathlib import Path
import subprocess
from horus.remote_rendering.v3_protocol import unpack_ros_diagnostic_frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cpp", required=True)
    parser.add_argument("--rust", required=True)
    args = parser.parse_args()
    fixture_path = Path(__file__).resolve().parents[2] / "contracts/fixtures/remote_protocol.json"
    fixture = json.loads(fixture_path.read_text())
    for command in ([args.cpp], [args.rust, str(fixture_path)]):
        result = subprocess.run(command, text=True, capture_output=True, check=True, timeout=20)
        metadata, image = unpack_ros_diagnostic_frame(bytes.fromhex(result.stdout.strip()))
        assert metadata.to_dict() == fixture["metadata"]
        assert image.hex() == fixture["image_hex"]
        print(json.dumps({"binary": command[0], "python_decoder": "passed"}))


if __name__ == "__main__":
    main()
