#!/usr/bin/env python3
"""Register real-model robots with URDF-backed bodies and Compass enabled.

Fetch the sample URDFs once:
    python3 python/examples/tools/fetch_robot_description_assets.py

Pair this with:
    python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models

Start Compass separately before connecting from HORUS MR:
    cd ~/compass
    source .venv/bin/activate
    compass voice-gateway serve --host 0.0.0.0 --port 8088

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/robot_description_compass_registration.py
"""

from robot_description_registration import run_registration


if __name__ == "__main__":
    run_registration(compass_enabled=True)
