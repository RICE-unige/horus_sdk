"""Keep the native helper fixture tied to Python's public registration methods."""
import importlib.util
import json
from pathlib import Path


def test_native_robot_helpers_contract():
    root = Path(__file__).resolve().parents[2]
    path = root / ".github/scripts/generate_robot_helpers_contract.py"
    spec = importlib.util.spec_from_file_location("robot_helpers_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    expected = json.loads((root / "contracts/fixtures/robot_helpers.json").read_text())
    assert module.build() == expected
