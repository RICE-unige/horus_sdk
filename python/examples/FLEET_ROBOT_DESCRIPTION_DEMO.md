# Heterogeneous fleet robot-description demo

Shows the robot-model upgrade end to end: the HORUS SDK pulls each robot's URDF straight
from a **normal ROS `robot_description` workflow** and renders the robot's **real visual
meshes** in MR — no per-robot URDF path hand-fed to the SDK, RViz-style.

Fleet (heterogeneous):

| Robot     | Kind     | Mesh format | Default body mode   |
|-----------|----------|-------------|---------------------|
| jackal    | wheeled  | STL         | `preview_mesh`      |
| go1       | legged   | DAE         | `runtime_high_mesh` |
| anymal_c  | legged   | DAE         | `preview_mesh`      |
| h1        | humanoid | STL         | `runtime_high_mesh` |

## What's new (this demo exercises it)

- **`source="topic"`** — the SDK reads the latched `/<ns>/robot_description` topic (the same
  source RViz uses), via a one-shot `ros2 topic echo` with transient-local QoS. `--source ros`
  reads the `robot_state_publisher` parameter instead. No `urdf_path` required.
- **`package://` mesh resolution** — resolves through the **ament index** (installed/sourced
  ROS packages) and falls back to a local **`mesh_root`** tree, so the demo runs with real
  meshes even when the description packages are *not* installed.
- **`body_mesh_mode`** per robot — `collision_only` / `preview_mesh` / `runtime_high_mesh`.

## Run it

All commands from the `horus_sdk` repo root, ROS 2 sourced.

```bash
# 1) One-time: fetch the four URDFs + their package:// meshes into a local mesh_root.
python3 python/examples/tools/fetch_robot_description_assets.py
#    -> writes python/examples/.local_assets/robot_descriptions/{*.urdf, meshes_root/...}
#    Use --skip-meshes instead if you have the ros-<distro>-*-description packages installed.

# 2) Bring up the ROS robot_description graph (robot_state_publisher + joint_state_publisher
#    per robot, namespaced, latched /<ns>/robot_description, frame_prefix "<ns>/").
ros2 launch python/examples/launch/fleet_robot_state_publishers.launch.py

# 3) In another terminal: register the fleet with HORUS MR.
PYTHONPATH=python:$PYTHONPATH python3 python/examples/fleet_robot_description_registration.py
#    --source ros            read the robot_state_publisher parameter instead of the topic
#    --body-mesh-mode MODE   force one mode for all robots
#    --no-meshes             register collision-only
#    --mesh-root PATH        override the package:// mesh fallback root
```

## Validate offline (no ROS graph, no HORUS bridge)

Resolves + bakes every robot from the local files and prints the baked manifest. Useful to
confirm the assets and the SDK pipeline before going on-device:

```bash
PYTHONPATH=python:$PYTHONPATH python3 python/examples/fleet_robot_description_registration.py --dry-run
```

Expected (mesh sizes vary with body mode):

```
jackal   : links= 13 joints= 12 collisions=  5 mesh_assets=  5 meshes=True  ...
go1      : links= 46 joints= 45 collisions= 38 mesh_assets= 13 meshes=True  ...
anymal_c : links= 78 joints= 77 collisions= 45 mesh_assets= 13 meshes=True  ...
h1       : links= 25 joints= 24 collisions= 23 mesh_assets= 20 meshes=True  ...
DRYRUN_OK
```

## Notes / troubleshooting

- **Jackal needs `xacro`** (provided by ROS) to expand `jackal.urdf.xacro`. The fetch tool
  writes an expanded `jackal.urdf` when xacro is available; otherwise the launch and resolver
  expand the xacro at runtime.
- **HORUS `RobotType` has no humanoid class**, so H1 registers as `LEGGED` (closest fit).
- **Frame naming**: `robot_state_publisher` uses `frame_prefix="<ns>/"` and the SDK robot is
  named `<ns>`, so its `prefixed` TF binding resolves frames to `<ns>/<link>` automatically.
- The `world -> <ns>/<root>` static transforms space the robots apart (helpful in RViz);
  HORUS anchors each robot in its own workspace, so they may be repositioned in MR.
- **On-device render** (bridge → Unity → Quest) is the live test to run on your hardware;
  everything up to the baked registration payload is validated by `--dry-run`.
