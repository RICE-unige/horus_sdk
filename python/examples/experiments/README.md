# HORUS Experiment Benchmarks

This folder contains the scaffold used to generate repeatable benchmark run
folders for HORUS performance characterization.

The basic run package is:

```text
results/<run_id>/
  run_manifest.json
  source_metrics.csv
  source_events.ndjson
  operator_metrics.csv
  failure_metrics.csv
  transport_manifest.json
  orchestrator_events.ndjson
  bridge_metrics.csv
  headset_metrics.csv
  webrtc_metrics.csv
  command_metrics.csv
  events.ndjson
  summary.json
  data_quality.json
```

Run the minimal baseline experiment from the SDK root:

```bash
cd ~/horus_sdk
./experiment 0
```

For headset runs, the script starts the bridge, then pauses before registration.
Use that pause to connect the HORUS app to the bridge and create/select the
workspace. Press Enter in the terminal only after the app is ready. Measurement
and workload timing starts after that confirmation.

Run another numbered experiment:

```bash
./experiment 1
./experiment 2
```

E10 is the camera-capacity sweep. It runs the same workload family at
increasing camera counts so the capacity limit is measured from comparable
conditions instead of inferred from a single fixed-load trial:

```bash
./experiment 10
```

The E10 ROS-compressed sweep conditions are:

```text
e10_camera_capacity_ros_01stream
e10_camera_capacity_ros_02stream
e10_camera_capacity_ros_04stream
e10_camera_capacity_ros_06stream
e10_camera_capacity_ros_08stream
```

Run an individual E10 condition when you need to repeat one load point:

```bash
./experiment e10_camera_capacity_ros_04stream
```

For a safe pilot before the full 120-second sweep:

```bash
./experiment 10 --duration 30 --warmup 10 --notes "E10 camera capacity pilot 01"
```

After the E10 runs finish, summarize camera delivery with:

```bash
PYTHONPATH=python python3 python/examples/experiments/horus_camera_capacity_report.py \
  --output results/e10_camera_capacity_summary.csv
```

E4 is a paired map-representation experiment. Run both conditions before using
the mesh-vs-pointcloud result in the paper:

```bash
./experiment 4
./experiment e4_pointcloud_map
```

The wrapper resolves `configs/e<N>_*.json`, creates the run folder, exports
the required environment variables for all child processes, starts the bridge
with priority scheduling, starts the headset recorder, registers the synthetic
robots and visualizations, runs the configured workload, stops the recorder,
exports headset-side metrics after measurement ends, and writes all metrics
into the run folder.

`duration_s` is the measured interval. `warmup_s` is recorded separately and
excluded from `summary.json` when `measurement_start` and `measurement_end`
events are present. The synthetic source runs for `warmup_s + duration_s`, so
the measured window starts only after the system is warm.

A run is valid for paper numbers only when `data_quality.json` reports
`"ok": true`. If a run is intentionally source-only, use `--no-headset`; that
explicitly disables the headset metric requirement.

Each experiment publishes normal HORUS SDK registrations. Synthetic robot
topics are under `/exp_robot_<N>/...`; shared map topics are under
`/horus/experiment/...`. This lets the MR app see the same structure it sees
from real SDK examples.

Useful options:

```bash
./experiment 0 --duration 30
./experiment 0 --ready-mode delay --ready-delay 60
./experiment 0 --ready-mode none
./experiment 0 --no-bridge
./experiment 0 --no-workload
./experiment 0 --no-headset --no-registration --no-transport
./experiment 0 --workload-command "python3 my_real_workload.py"
./experiment 0 --ros-domain-id 51
```

Dry-run the registration and run folder without launching ROS processes:

```bash
./experiment 0 --dry-run
```

Transport profiles:

```bash
./experiment 2 --transport-profile local
./experiment 2 --transport-profile direct --robot-host arancino --workload-host arancino
./experiment 2 --transport-profile cloud --robot-host arancino --workload-host arancino --cloud-host googlecloud
```

The default `auto` profile starts a local connector baseline path only for
WebRTC/H.264 configs. A real direct LAN or cloud transport trial must place the
source publisher on the robot side too. Use `--workload-host <ssh-host>` for the
built-in synthetic workload, or use `--no-workload` only when a real camera
source is already publishing on the robot host.

When running from WSL on this workstation, `./experiment` automatically uses
Windows OpenSSH at `/mnt/c/Windows/System32/OpenSSH/ssh.exe` if available, so
the `arancino`, `arancina`, `poke`, and `googlecloud` aliases from the Windows
SSH config are usable. Override this with `--ssh-command ssh` if WSL SSH is
configured separately.

For a real robot source already running on `arancino`:

```bash
./experiment 2 --transport-profile direct --robot-host arancino --no-workload
```

The built-in connector profile launches the first camera stream and records
what it did in `transport_manifest.json`. For multi-camera connector farms,
pass additional commands explicitly:

```bash
./experiment 2 \
  --remote-command "robot1=ssh arancino 'cd ~/horus_connector && ./horus launch robot --no-monitor'" \
  --remote-command "robot2=ssh arancina 'cd ~/horus_connector && ./horus launch robot --no-monitor'"
```

Failure injection is config-driven. E9 restarts the bridge at the configured
time unless disabled:

```bash
./experiment 9 --no-failure-injection
```

The synthetic workload is intentionally realistic but still labeled synthetic:

- camera streams publish dynamic RGB or JPEG frames at the configured
  resolution and rate;
- point clouds contain structured XYZRGB geometry with the configured point
  count and point step;
- mesh maps publish chunked `visualization_msgs/Marker` triangle lists so old
  chunks remain visible while updates arrive;
- E8 publishes multi-operator presence and control lease snapshots using the
  same HORUS multi-operator topic contracts;
- E9 records bridge restart events in `orchestrator_events.ndjson`.
- E10 records a camera-stream capacity curve. Each condition registers one
  synthetic robot per camera stream and publishes 1280x720 JPEG-compressed
  frames at 30 fps over the ROS-compressed image path.

Manual control is still available:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=<domain>

PYTHONPATH=python python3 python/examples/experiments/horus_experiment_control.py \
  start --run-id E2_webrtc_vs_ros_1280x720_30fps_4streams_lan_r01 \
  --experiment E2_webrtc_vs_ros \
  --condition 1280x720_30fps_4streams_lan

PYTHONPATH=python python3 python/examples/experiments/horus_experiment_control.py stop

PYTHONPATH=python python3 python/examples/experiments/horus_experiment_control.py \
  export --run-id E2_webrtc_vs_ros_1280x720_30fps_4streams_lan_r01
```

If you launch the ROS 2 bridge or connector outside `./experiment`, set:

```bash
export HORUS_EXPERIMENT_RESULTS_DIR=/path/to/results/<run_id>
export HORUS_EXPERIMENT_RUN_ID=<run_id>
export HORUS_EXPERIMENT=<experiment>
export HORUS_EXPERIMENT_CONDITION=<condition>
```

The benchmark configs in `configs/` are starting points for the full E1-E9
matrix. They define conditions and expected stream profiles; actual paper
numbers must come from measured run folders, not from these config values.
