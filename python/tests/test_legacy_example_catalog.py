"""Guardrails for documented legacy example command paths."""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path


SDK_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = SDK_ROOT.parent
LEGACY_README = SDK_ROOT / "examples" / "legacy" / "README.md"


def test_documented_legacy_example_scripts_exist():
    text = LEGACY_README.read_text(encoding="utf-8")
    documented = sorted(set(re.findall(r"python/examples/legacy/[^\s`]+\.py", text)))
    assert documented, "legacy README should document runnable example scripts"

    missing = [path for path in documented if not (REPO_ROOT / path).is_file()]
    assert missing == []


def test_legacy_example_catalog_mentions_current_core_workflows():
    text = LEGACY_README.read_text(encoding="utf-8")
    required = [
        "sdk_typical_ops_demo.py",
        "sdk_robot_description_demo.py",
        "sdk_robot_description_tutorial_demo.py",
        "sdk_multi_operator_host_demo.py",
        "sdk_legged_ops_demo.py",
        "sdk_fake_semantic_perception_demo.py",
        "fake_tf_ops_suite.py",
        "fake_tf_robot_description_suite.py",
        "fake_tf_drone_ops_suite.py",
        "fake_tf_legged_ops_suite.py",
        "fake_3d_map_publisher.py",
        "fake_octomap_publisher.py",
    ]
    missing = [name for name in required if name not in text]
    assert missing == []


def test_documented_sdk_legacy_examples_expose_help_without_starting_runtime():
    scripts = [
        "sdk_fake_semantic_perception_demo.py",
        "sdk_hospital_carter_live_demo.py",
        "sdk_legged_ops_demo.py",
        "sdk_multi_operator_host_demo.py",
        "sdk_registration_demo.py",
        "sdk_robot_description_demo.py",
        "sdk_robot_description_tutorial_demo.py",
        "sdk_single_robot_flat_demo.py",
        "sdk_stereo_registration_demo.py",
        "sdk_typical_ops_demo.py",
    ]
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SDK_ROOT)

    for script in scripts:
        path = SDK_ROOT / "examples" / "legacy" / script
        result = subprocess.run(
            [sys.executable, str(path), "--help"],
            text=True,
            capture_output=True,
            timeout=10,
            env=env,
            check=False,
        )
        assert result.returncode == 0, result.stderr or result.stdout
        assert "usage:" in result.stdout.lower()
