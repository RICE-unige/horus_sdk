# Heterogeneous fleet robot-description demo

Shows the robot-model upgrade end to end: the HORUS SDK registers a 5-robot showroom
fleet with each robot's **real visual meshes** in MR. By default the static body models
come from the cached local URDF assets, while the ROS graph publishes only one base
anchor frame per robot. Use `--source topic` and `publish_full_tf:=true` when you
specifically want to exercise the RViz-style `robot_description` topic path.

Showroom fleet (heterogeneous):

| Robot     | Kind     | Mesh format | Default body mode       |
|-----------|----------|-------------|-------------------------|
| g1        | humanoid | STL         | `runtime_high_mesh`     |
| h1        | humanoid | STL         | `runtime_high_mesh`     |
| anymal_c  | legged   | DAE         | `runtime_high_mesh`     |
| spot      | legged   | OBJ/STL     | `runtime_high_mesh`     |
| jackal    | wheeled  | STL         | `runtime_high_mesh`     |

## What's new (this demo exercises it)

- **`source="local"` default** — the SDK reads the cached URDF/xacro files fetched by the
  asset tool. This avoids creating ten extra ROS CLI participants just to register static
  showroom bodies.
- **Live ROS description modes** — `--source topic` reads the latched
  `/<ns>/robot_description` topic, and `--source ros` reads the
  `robot_state_publisher` parameter. Use these when you want to test the transport path
  itself rather than the showroom.
- **`package://` mesh resolution** — resolves through the **ament index** (installed/sourced
  ROS packages) and falls back to a local **`mesh_root`** tree, so the demo runs with real
  meshes even when the description packages are *not* installed.
- **Full visual bodies by default** — the showroom sends high-detail visual meshes for
  each robot. Use `--body-mesh-mode preview_mesh` only when you intentionally want a
  lighter diagnostic run.
- **`body_mesh_mode`** per robot — `collision_only` / `preview_mesh` / `runtime_high_mesh`.
- **Showroom floor alignment** — the launch publishes measured root-frame Z offsets so each
  robot stands on the workspace floor instead of sinking leg links below the floor plane.
- **Base-anchor TF by default** — the launch publishes only 5 static base anchors.
  This avoids hundreds of URDF link frame markers/labels in HORUS MR. Use
  `publish_full_tf:=true` only for RViz-style TF validation.
- **Robot Manager setup** — this showcase enables status and data visualization only. It
  intentionally hides teleoperation and task controls because the demo ROS graph publishes
  model/TF data, not robot command interfaces.

## Run it

All commands from the `horus_sdk` repo root, ROS 2 sourced.

```bash
# 1) One-time: fetch the showroom URDFs + their package:// meshes into a local mesh_root.
python3 python/examples/tools/fetch_robot_description_assets.py
#    -> writes python/examples/.local_assets/robot_descriptions/{*.urdf, meshes_root/...}
#    Use --skip-meshes instead if you have the ros-<distro>-*-description packages installed.

# 2) Bring up the base-anchor TF graph.
ros2 launch python/examples/launch/fleet_robot_state_publishers.launch.py

# Optional RViz-style validation path: publishes /<ns>/robot_description and the
# full prefixed URDF TF tree. This is intentionally not the default HORUS MR path.
ros2 launch python/examples/launch/fleet_robot_state_publishers.launch.py publish_full_tf:=true

# 3) In another terminal: register the fleet with HORUS MR.
PYTHONPATH=python:$PYTHONPATH python3 python/examples/fleet_robot_description_registration.py
#    --source topic          read /<ns>/robot_description instead of local files
#    --source ros            read the robot_state_publisher parameter instead of local files
#    --body-mesh-mode MODE   force one mode for all robots
#    --visual-mesh-triangle-budget N
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
g1       : links= 40 joints= 39 collisions= 37 mesh_assets= 30 meshes=True  ...
h1       : links= 25 joints= 24 collisions= 23 mesh_assets= 20 meshes=True  ...
anymal_c : links= 78 joints= 77 collisions= 45 mesh_assets= 13 meshes=True  ...
spot     : links= 14 joints= 13 collisions= 13 mesh_assets= 13 meshes=True  ...
jackal   : links= 13 joints= 12 collisions=  5 mesh_assets=  5 meshes=True  ...
DRYRUN_OK
```

## Notes / troubleshooting

- **Jackal needs `xacro`** (provided by ROS) to expand `jackal.urdf.xacro`. The fetch tool
  writes an expanded `jackal.urdf` when xacro is available; otherwise the launch and resolver
  expand the xacro at runtime.
- **HORUS `RobotType` has no humanoid class**, so G1 and H1 register as `LEGGED` (closest fit).
- **Frame naming**: the default launch publishes `world -> <ns>/<base>`. The optional
  `publish_full_tf:=true` path uses `robot_state_publisher` with `frame_prefix="<ns>/"`
  so full URDF frames become `<ns>/<link>`.
- The `world -> <ns>/<urdf_root>` static transforms space the robots in a 2-row showroom
  grid and lift each URDF root enough that wheels/feet sit on the floor plane.
- If CycloneDDS reports `Failed to find a free participant index`, stop the old launch and
  restart this updated launch. The default avoids launching one `robot_state_publisher`
  per robot.
- **On-device render** (bridge → Unity → Quest) is the live test to run on your hardware;
  everything up to the baked registration payload is validated by `--dry-run`.
