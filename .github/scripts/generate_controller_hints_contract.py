"""Generate synthetic registration fixtures; never opens recordings or devices."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'python'), str(ROOT / 'python/tests')]
from horus import ControllerHint, ControllerHintProfile, ControllerHints, TeleopHintMode
from horus.robot import Robot, RobotType
from test_semantic_config_helpers import _build_config


def fixture():
    hints = ControllerHints(tuple(ControllerHintProfile(mode, (
        ControllerHint('left_stick_y', 'move_forward', 'Forward/back'),
        ControllerHint('right_secondary', 'lamp', 'Inspection light'),
        ControllerHint('left_primary', 'tool_mode', 'Mode: grasp'),
    ), 'inspection') for mode in TeleopHintMode), state_topic='/hint_demo/controller_hints')
    robot = Robot('hint_demo', RobotType.WHEELED)
    robot.configure_teleop(raw_input_topic='/hint_demo/joy')
    robot.configure_controller_hints(hints)
    result = _build_config(robot)
    result['timestamp'] = 0.0
    return result


if __name__ == '__main__':
    output = ROOT / 'contracts/fixtures/controller_hints_registration.json'
    output.write_text(json.dumps(fixture(), sort_keys=True, indent=2) + '\n')
    unicode = ControllerHints((ControllerHintProfile(TeleopHintMode.IMMERSIVE,
        (ControllerHint('right_primary', 'tool', 'Caf\u00e9 \U0001f680'),)),)).to_dict()
    output.with_name('controller_hints_unicode.json').write_text(json.dumps(unicode, indent=2) + '\n')
    print(output)
