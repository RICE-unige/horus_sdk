# HORUS Experiment Benchmarks

This folder contains the scaffold used to generate repeatable benchmark run
folders for HORUS performance characterization.

The basic run package is:

```text
results/<run_id>/
  run_manifest.json
  source_metrics.csv
  bridge_metrics.csv
  headset_metrics.csv
  webrtc_metrics.csv
  command_metrics.csv
  events.ndjson
  summary.json
```

Create a smoke run folder:

```bash
cd ~/horus_sdk
PYTHONPATH=python python3 python/examples/experiments/horus_benchmark_suite.py \
  --config python/examples/experiments/configs/e0_smoke.json
```

Start and stop HORUS MR headset recording over ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=<domain>

PYTHONPATH=python python3 python/examples/experiments/horus_experiment_control.py \
  start --run-id E2_webrtc_vs_ros_1280x720_30fps_4streams_lan_r01 \
  --experiment E2_webrtc_vs_ros \
  --condition 1280x720_30fps_4streams_lan

PYTHONPATH=python python3 python/examples/experiments/horus_experiment_control.py stop
```

When running the ROS 2 bridge or connector during an experiment, set:

```bash
export HORUS_EXPERIMENT_RESULTS_DIR=/path/to/results/<run_id>
export HORUS_EXPERIMENT_RUN_ID=<run_id>
export HORUS_EXPERIMENT=<experiment>
export HORUS_EXPERIMENT_CONDITION=<condition>
```

The benchmark configs in `configs/` are starting points for the full E1-E9
matrix. They define conditions and expected stream profiles; actual paper
numbers must come from measured run folders, not from these config values.
