---
title: Controller Labels
---

HORUS shows small controller-anchored labels during MiniMap, immersive, Follow-Leader and Teach teleoperation. A label describes an assigned input; it does not create a binding or grant control. Unassigned inputs have no label. Old registrations continue to show known built-in motion, deadman and Teach stop controls.

## Declare Labels

```python
from horus import ControllerHint, ControllerHintProfile, ControllerHints, TeleopHintMode

profiles = (
    ControllerHintProfile(TeleopHintMode.MINIMAP, (
        ControllerHint("left_stick_y", "move_forward", "Forward/back"),
        ControllerHint("right_secondary", "lamp", "Toggle lamp"),
    ), "inspection"),
)
robot.configure_controller_hints(ControllerHints(profiles))
```

The inspection backend must already handle the B button in the configured raw Joy stream. Describing `lamp` alone cannot switch a physical lamp. Only inputs actually forwarded by the active mode are eligible. Standard teleoperation forwards configured raw Joy; custom-passthrough does not invent standard Twist/deadman functions. Teach stick/face-button descriptions require negotiated v2 features. Descriptions never alter velocities, leases, clutch settings, engagement or motor commands.

Modes: `minimap`, `immersive`, `follow_leader`, `teach`. One profile per mode, at most four profiles. Reusing one profile's binding tuple for multiple modes is supported. Inputs are `left_` / `right_` plus `primary`, `secondary`, `trigger`, `grip`, `stick_x`, `stick_y`, or `stick_click`. On Quest, left primary/secondary are X/Y and right primary/secondary are A/B.

Each binding has a stable lowercase `action_id` and a single-line label of at most 24 UTF-16 code units. Empty labels hide custom descriptions, not known app-owned functions or stop/run chords. Labels are plain text, not markup. IDs are at most 64 ASCII letters/digits/underscores and start with a lowercase letter. Metadata is optional; `robot.configure_controller_hints(None)` removes it.

Known app action IDs: `move_forward`, `move_lateral`, `move_vertical`, `turn`, `deadman`, `hold`, `clutch`, `disengage`. A description may rename that same action, but cannot replace an app-owned function with another action. Stop/run chords remain visible alongside a single-button action. For example X can describe an adapter action while X+A still explicitly reads Disengage. Any active Compass button functions remain annotated too. Empty custom labels cannot remove those chord/Compass annotations.

Teach automatically derives grasp descriptions from the existing `TeachConfig.grasp_bindings`. To customize one, use its **same action ID** in the Teach hint profile. No change to the Teach configuration digest, input schema or adapter mapping is required.

## Dynamic Action Descriptions

Use a dedicated display-state topic when a description depends on backend state. A static label should describe the function, such as `Toggle lamp`, rather than assume the last state remains current.

```python
from horus import ControllerHintContext
from std_msgs.msg import String

hints = ControllerHints(profiles, state_topic="/inspection/controller_hints",
                        state_timeout_seconds=2.0)
robot.configure_controller_hints(hints)
revision = 0
publisher = node.create_publisher(String, hints.state_topic, 1)

# Subscribe with your existing ROS node to hints.context_topic.
def on_context(message):
    global revision
    context = ControllerHintContext.from_json(message.data)
    revision += 1
    update = hints.reply(context, robot_name=robot.name,
                         labels={"lamp": "Lamp on" if lamp_enabled else "Lamp off"},
                         revision=revision)
    publisher.publish(String(data=update.to_json()))  # publisher: hints.state_topic
```

HORUS publishes a small context heartbeat at 1 Hz while that robot/mode is active. Replies must echo robot, configuration hash, mode, profile, fresh context UUID and heartbeat sequence, with an increasing revision. Updates may change only the text of actions declared in that profile, not add inputs or commands. Wrong-context, old-sequence, replayed, unknown-action and oversized replies are discarded. Text expires after the local receipt timeout (0.5-10 seconds), then falls back to the static function description. No synchronized host/headset clock is required.

Use a dedicated topic. HORUS refuses to take over another component's subscriber/publisher. Switching robot, mode, Teach session, connection or leaving teleoperation invalidates the old context and clears its labels. Labels also hide for untracked controllers and when application focus is lost. No controller mesh, application font/render setting, raycast surface or interaction handler is changed.

## Other Languages

C++ and Rust expose the same typed controls, modes, profile/configuration, context and label-update contracts. Registration uses the same optional `controller_hints` JSON field and configuration digest.

```cpp
horus::ControllerHints hints;
hints.profiles = {{horus::TeleopHintMode::Minimap,
    {{horus::ControllerControl::RightSecondary, "lamp", "Toggle lamp"}}, "inspection"}};
robot.configure_controller_hints(hints);
// ControllerHintContext::from_json(payload); hints.reply(context, robot_name, labels, revision)
```

```rust
use horus::{ControllerControl, ControllerHint, ControllerHintProfile, ControllerHints, TeleopHintMode};
let hints = ControllerHints {
    profiles: vec![ControllerHintProfile::new(TeleopHintMode::Minimap,
        vec![ControllerHint::new(ControllerControl::RightSecondary, "lamp", "Toggle lamp")?], "inspection")?],
    ..Default::default()
};
robot.configure_controller_hints(&hints)?;
// ControllerHintContext::from_json(payload)?; hints.reply(&context, robot_name, labels, revision)?
```

Headers: `horus/controller_hints.hpp`; Rust module: `horus::controller_hints`. C++ updates provide `to_json()`, Rust `to_json()?`, Python `to_json()`. Helpers validate/build messages only and never create ROS nodes or start controllers implicitly.

## Synthetic Examples

From an SDK checkout with ROS 2 sourced:

```bash
PYTHONPATH=python:$PYTHONPATH python3 python/examples/controller_hints_demo.py
```

This registers one virtual rover, TF and a low-rate synthetic camera. Hold the configured grip deadman and use the sticks to move the virtual robot. B toggles the virtual lamp and its label/camera color. There is no hardware control. MiniMap and immersive modes use the same declarations; Follow-Leader uses the active leader's profile when that feature is active.

```bash
PYTHONPATH=python:$PYTHONPATH python3 python/examples/humanoid_teach_demo.py --continuous
```

This existing synthetic humanoid adapter describes its actual grasp/pinch bindings. X+A disengagement and B Hold remain app-owned. Unused sticks and buttons stay unlabelled. Native contract, registration, ROS serialization/routing and app parse/resolver/lifecycle tests cover the shared synthetic fixtures; physical controller readability/placement still needs device acceptance and is not established by headless tests.
