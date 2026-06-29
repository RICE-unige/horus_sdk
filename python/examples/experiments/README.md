# HORUS Core Platform Experiments

This folder contains the benchmark infrastructure for the i-RIM HORUS system
paper. The scope is the local/core HORUS MR-ROS 2 platform: the Quest runtime,
the HORUS ROS 2 bridge, SDK registration, local sensor/media streams, shared
maps, tasking, teleoperation, and multi-operator leases under controlled LAN
conditions.

Remote VPN/cloud relay experiments belong to the RA-L work. Do not use WAN,
VPN, Tailscale, Zenoh relay, or country-to-country transport numbers in the
i-RIM evaluation except as future work.

## What a Run Produces

Each run creates the core files below. Optional files are created only when the
workload exercises that path.

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
  headset_events.ndjson
  webrtc_metrics.csv        # optional connector diagnostic, not used for i-RIM
  command_metrics.csv
  clock_sync.json
  events.ndjson
  summary.json
  derived_summary.json
  data_quality.json
```

Use a run for paper numbers only when `data_quality.json` has
`"measurement_valid": true`. Interpret `"within_envelope": true` as a workload
inside the usability envelope. Interpret `"degraded": true` as a complete,
valid measurement at or beyond the operating boundary.

Source-only diagnostic runs are valid only when they are explicitly run with
`--no-headset`; do not mix them with headset performance results.

## One-Time Setup

Use the `experiments` branch in both repositories.

```bash
cd ~/horus_sdk
git switch experiments
chmod +x ./experiment
```

In Unity, open the HORUS MR project from its `experiments` branch and deploy or
run the Quest build from that branch. The MR experiment recorder is inactive by
default. It only subscribes to `/horus/experiments/control` after a robot
registration includes:

```json
"workspace_config": {
  "experiment": {
    "enabled": true,
    "contract_version": "experiment.v1"
  }
}
```

The experiment registration script sets this flag for the benchmark robots, so
normal SDK examples should not activate experiment logging.

Before restarting a measurement session, rebuild or redeploy the HORUS MR app
from the current `experiments` branch. An older APK can still connect to the
bridge, but it will not have the current experiment recorder fields and the run
will fail the headset data-quality checks.

## Standard Run Flow

For headset runs, `./experiment` starts the bridge first and then pauses. During
that pause:

1. Put on the Quest.
2. Open HORUS.
3. Connect to the bridge shown by the script.
4. Create or select the workspace.
5. Wait until the workspace is stable.
6. Press Enter in the terminal.

Only after this confirmation does the script register the synthetic robot
workload, start the headset recorder, warm up the streams, and begin the
measured interval.

The script sets `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, `PYTHONPATH`, run IDs,
result paths, and experiment metadata itself.

After the measured interval, keep the headset app connected until the script has
finished the headset export step. The preferred export path is the
`/horus/experiments/status` topic; ADB is only a fallback. Do not close HORUS
immediately after the timer ends, because the host still needs the final
headset metrics and clock-sync response.

## Quick Validation

Run this before a full session:

```bash
cd ~/horus_sdk
./experiment 0 --dry-run --duration 2 --warmup 0 --notes "dry registration check"
```

Then run a source-only validation that does not require the Quest:

```bash
./experiment 0 \
  --no-headset --no-registration --no-bridge --no-transport \
  --ready-mode none \
  --duration 2 --warmup 0 \
  --notes "source-only validation"
```

Check:

```bash
cat results/<run_id>/data_quality.json
cat results/<run_id>/summary.json
cat results/<run_id>/derived_summary.json
cat results/<run_id>/clock_sync.json
```

A valid headset run should have:

- `data_quality.json` with `"measurement_valid": true`
- at least two `clock_sync.json` samples, one before measurement and one after
  measurement
- `headset_metrics.csv` with `frame/render` rows in the measured window
- no missing required files in `data_quality.json`

The headset `extra_json` field includes runtime availability flags such as
`gpu_time_ms_available`, `dropped_frames_available`,
`ovr_app_target_fps_available`, `ovr_cpu_level_available`,
`ovr_gpu_level_available`, and `ovr_battery_temp_c_available`. Use a runtime
metric in the paper only when its matching availability flag is `true`. This is
important on Quest/OpenXR because some Meta runtime fields are not exposed on
all builds.

## Experiment Matrix

Use five repetitions for the primary claims: E0, E6, E7, and E10. Use at least
three repetitions for secondary runs: E1, E3, E4, E5, E8, and E9. Change only
the notes or run ID between repetitions.

Keep the standard timing unless a section explicitly says otherwise:

```text
Warmup:       30 s
Measurement: 120 s
```

### E0 Baseline Latency

Purpose: basic local stack latency with minimal load.

Metrics: registration latency, TF freshness, state update latency, task/goal
latency, teleop command latency, lease acquire/release, workspace join/replay.

```bash
./experiment 0 --duration 120 --warmup 30 --notes "E0 baseline r01"
```

### E1 Camera Transport, ROS Path

Purpose: overview/minimap camera behavior on the normal ROS image path.

Metrics: source FPS, bridge-forwarded FPS, Quest received FPS, displayed FPS,
latency, dropped frames, Quest FPS, bridge CPU/memory.

```bash
./experiment 1 --duration 120 --warmup 30 --notes "E1 ROS camera r01"
```

### E2 Connector/WebRTC Diagnostic

Status: out of scope for the current i-RIM core-platform paper.

Do not run E2 for the i-RIM dataset. It uses the connector/WebRTC media path,
which belongs to the later RA-L remote-transport work. The i-RIM paper should
use E1 and E10 for camera/media behavior on the core local HORUS MR-ROS 2
platform.

The launcher now refuses connector transport profiles unless
`--allow-connector-transport` is passed explicitly. That override is only for
separate connector diagnostics, not for paper numbers.

Next i-RIM experiment after E1 is E3.

### E3 PointCloud2 Capacity

Purpose: bounded-space point-cloud capacity.

Metrics: source rate, payload size, Quest receive/display rate, parse/apply
time, Quest FPS, memory, bridge CPU.

```bash
./experiment 3 --duration 120 --warmup 30 --notes "E3 pointcloud capacity r01"
```

### E4 Map Representation

Purpose: justify the preferred 3D shared-map representation.

Run both conditions:

```bash
./experiment 4 --duration 120 --warmup 30 --notes "E4 triangle mesh r01"
./experiment e4_pointcloud_map --duration 120 --warmup 30 --notes "E4 pointcloud map r01"
```

Report mesh map versus point-cloud map using displayed rate, payload size,
parse/apply time, Quest FPS, memory, and bridge CPU. The expected conclusion is
that a triangle-shell mesh is the preferred 3D shared-map path, while
PointCloud2 is appropriate for smaller bounded spaces.

### E5 Map Update Behavior

Purpose: verify full-map persistence and incremental chunk updates.

```bash
./experiment 5 --duration 120 --warmup 30 --notes "E5 map update r01"
```

The complete map should remain visible while new chunks arrive.

### E6 Control Under Sensor Load

Purpose: prove that visualization load does not destroy control usability.
This is the realistic control workload: TF/odom, navigation paths, one
compressed overview camera, a triangle-shell shared mesh map, and goal/control
traffic. It intentionally does not combine a PointCloud2 map with the mesh map,
because HORUS treats PointCloud2 maps as a bounded-space diagnostic path rather
than the recommended 3D shared-map representation.

```bash
./experiment 6 --duration 120 --warmup 30 --notes "E6 control under load r01"
```

Report teleop latency p50/p95/p99, task/goal latency, lease latency, TF
freshness, dropped/stale messages, Quest FPS, and bridge CPU/memory.

Use E3 and the E4 pointcloud-map condition for the measured pointcloud evidence:
those runs show the pointcloud capacity and why the triangle-shell mesh path is
preferred for large shared maps.

### E7 Multi-Robot Scaling

Purpose: operating envelope across heterogeneous/synthetic robots.
This run is intentionally a robot-count scaling workload, not a pointcloud
stress test: it uses TF/odom, navigation paths, robot state, and four overview
camera streams across eight synthetic robots. Use E3 and the E4 pointcloud-map
condition for pointcloud limits.

```bash
./experiment 7 --duration 120 --warmup 30 --notes "E7 multi-robot r01"
```

Report the result as an operating envelope, for example:

```text
Under profile X, HORUS stayed stable up to N robots while keeping Quest FPS
above Y and command p95 latency below Z.
```

### E8 Multi-Operator Scaling and Leases

Purpose: local multi-operator state consistency and lease gating.

```bash
./experiment 8 --duration 120 --warmup 30 --notes "E8 multi-operator r01"
```

The workload publishes multi-operator presence and lease-state snapshots using
the HORUS multi-operator topic contracts. Measure join/replay time, lease
acquire/release latency, denied-command events, state consistency, bridge
CPU/memory, and Quest FPS per headset when multiple headsets are available.

### E9 Local Failure and Recovery

Purpose: compact recovery characterization.

```bash
./experiment 9 --duration 120 --warmup 30 --notes "E9 bridge restart r01"
```

By default E9 restarts the local bridge at the configured failure time. Disable
failure injection only for diagnostics:

```bash
./experiment 9 --no-failure-injection --duration 120 --warmup 30
```

Report detection time, stale-state duration, recovery time, command blocking,
registry replay behavior, and whether manual intervention was required.

### E10 Camera Capacity Sweep

Purpose: answer how many concurrent ROS camera streams the local platform can
handle before frame delivery or Quest frame rate breaks down.

```bash
./experiment 10 --notes "E10 staged camera capacity r01"
```

This is a single staged run. It registers eight synthetic robots once, then
enables camera streams during the measured interval:

```text
0-60 s:     1 stream
60-120 s:   2 streams
120-180 s:  4 streams
180-240 s:  6 streams
240-300 s:  8 streams
```

All streams use `1280x720`, `30 fps`, and the ROS compressed image path. Keep
the default `300 s` duration unless you are doing a source-only diagnostic.

The old multi-run sweep is retained only as a fallback diagnostic:

```bash
./experiment camera-capacity-sweep --duration 120 --warmup 30
```

### PointCloud Map Follow-Up

The current i-RIM dataset stops at E10, then uses the E4 pointcloud-map
condition as the focused pointcloud-map comparison against the triangle-shell
mesh path:

```bash
./experiment e4_pointcloud_map --duration 120 --warmup 30 --notes "E4 pointcloud map r01"
```

E11/E12 local map-capacity sweeps are retained as diagnostics, but they are not
part of the current paper run.

## Data Quality Gates

Use these targets before interpreting a run:

```text
Quest FPS                    >= 60 FPS
Teleop command latency p95   <= 150 ms
Task/goal latency p95        <= 500 ms
TF freshness p95             <= 100 ms
Overview displayed FPS       >= 15 FPS
Teleop displayed FPS         >= 30 FPS
No unbounded queue growth
```

These are usability thresholds, not hard claims. If a workload fails one, report
that workload as the edge of the operating envelope.

## Generate Tables and Summaries

Generate the aggregate paper report:

```bash
cd ~/horus_sdk
HORUS_SDK_NO_BANNER=1 PYTHONPATH=python \
python3 python/examples/experiments/horus_paper_report.py \
  --output-dir results/paper_report
```

Outputs:

```text
results/paper_report/paper_run_index.csv
results/paper_report/paper_metric_summary.csv
results/paper_report/paper_artifact_index.csv
results/paper_report/paper_report.md
results/paper_report/tables/*.csv
results/paper_report/tables/*.md
results/paper_report/figures/*.svg
```

The generated tables and figures are conservative. They use only measured rows
where `measurement_valid` is true. If an experiment group has not been collected
yet, the corresponding table/figure is created as an empty placeholder instead
of inventing a value.

Generate the camera-capacity table:

```bash
HORUS_SDK_NO_BANNER=1 PYTHONPATH=python \
python3 python/examples/experiments/horus_camera_capacity_report.py \
  --output results/e10_camera_capacity_summary.csv
```

Use only rows where `measurement_valid` is true for final paper numbers.

## Paper Result Mapping

Use the generated files to build these paper artifacts:

| Paper artifact | Source runs |
| --- | --- |
| Experimental setup table | `paper_run_index.csv` plus hardware/software notes |
| Baseline latency table/figure | E0, `tables/baseline_latency.*`, `figures/baseline_latency.svg` |
| Camera/media table/figure | E1, E10, `tables/camera_media.*`, `figures/camera_media.svg` |
| Sensor/map capacity table/figure | E3, E4, E5, `tables/sensor_map_capacity.*`, `figures/sensor_map_capacity.svg` |
| Control-under-load table/figure | E6, `tables/control_under_load.*`, `figures/control_under_load.svg` |
| Robot/operator scaling table/figure | E7, E8, `tables/scaling.*`, `figures/scaling.svg` |
| Failure/recovery table/figure | E9, `tables/failure_recovery.*`, `figures/failure_recovery.svg`, plus `orchestrator_events.ndjson` |
| Pointcloud-map comparison | E4 pointcloud-map condition plus E4 mesh condition, `tables/sensor_map_capacity.*`, `figures/sensor_map_capacity.svg` |

For the paper discussion, use `paper_artifact_index.csv` as a coverage check:
any row with zero matched metric rows means that experiment still needs a
measurement-valid run before it can support a numeric claim.

## Analysis and Discussion Workflow

Use this order when converting runs into paper-ready numbers:

1. Filter to valid runs.
   Use `paper_run_index.csv` and keep only rows where `measurement_valid` is `True`.
   Keep source-only validation runs out of headset performance tables.

2. Confirm coverage.
   Open `paper_artifact_index.csv`. Every table or figure used in the paper
   should have at least one matched metric row for the relevant experiment.

3. Report distributions, not only averages.
   For latency and freshness metrics, use p50, p95, and p99 from
   `paper_metric_summary.csv`. For FPS and stream rates, report measured source,
   bridge, headset receive, and displayed rates where those rows are present.

4. Define the operating envelope.
   For E6, E7, E8, and E10, describe the highest workload that still satisfies
   the usability targets. If the next workload fails, report it as the observed
   degradation point instead of smoothing it away.

5. Compare alternative data paths directly.
   Use E10 for ROS camera stream-count capacity and E4 mesh versus E4
   point-cloud map for 3D shared-map representation. Keep the claim tied to
   measured FPS, latency, payload size, CPU/memory, and dropped/stale rows.

6. Write the discussion from measured bottlenecks.
   Use stale-message counts, bridge CPU/memory, Quest FPS, displayed rate, and
   queue-growth indicators to explain whether the limiting factor was transport,
   parsing, decoding, texture upload, rendering, or headset frame budget.

Do not fill missing values by hand. If a required row is absent, rerun that
experiment or state that the corresponding claim is not supported by the
collected data.

## What Not To Use for i-RIM

The orchestrator still has direct/cloud transport options for later RA-L work,
but those are not part of this i-RIM core-platform characterization:

```text
horus_connector
Zenoh relay/hub
VPN/Tailscale
cloud relay
WAN latency/jitter/loss
Genoa-Napoli or international remote operation
```

Keep those experiments separate so the i-RIM paper remains focused on the
measured local HORUS MR-ROS 2 platform.
