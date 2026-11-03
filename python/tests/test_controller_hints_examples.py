import importlib.util
from pathlib import Path
from test_semantic_config_helpers import _build_config


def test_non_teach_demo_describes_real_local_joy_binding_only():
    path = Path(__file__).resolve().parents[1] / 'examples/controller_hints_demo.py'
    spec = importlib.util.spec_from_file_location('controller_hint_fixture', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    robot, view, hints = module.make_registration()
    payload = _build_config(robot)
    assert payload['controller_hints'] == hints.to_dict()
    assert payload['control']['teleop']['raw_input_topic'] == '/hint_demo/joy'
    assert len(hints.profiles) == 3
    assert all(len(p.bindings) == 3 for p in hints.profiles)
    assert all(p.bindings[-1].control.value == 'right_secondary' for p in hints.profiles)
