import csv
import json
from pathlib import Path
import subprocess
import sys

from horus.experiments.analysis import (
    aggregate_runs,
    find_saturation_knee,
    summarize_numeric_csv,
    validate_run_directory,
    write_summary,
)
from horus.experiments.manifest import ExperimentManifest, RunIdentity
from horus.experiments.metrics import (
    CLOCK_MONOTONIC_ANCHOR_ENV,
    CLOCK_WALL_ANCHOR_ENV,
    CsvMetricWriter,
    clock_anchor_env,
    source_metric_stream_seen,
)


def test_manifest_saves_run_id(tmp_path: Path):
    manifest = ExperimentManifest.from_identity(
        RunIdentity("run_001", "E0_smoke", "condition", 1),
        duration_s=1,
        warmup_s=0,
    )

    path = manifest.save(tmp_path)

    assert path.exists()
    assert '"run_id": "run_001"' in path.read_text(encoding="utf-8")


def test_csv_metric_writer_adds_common_fields(tmp_path: Path):
    path = tmp_path / "source_metrics.csv"
    with CsvMetricWriter(
        path,
        run_id="run_001",
        experiment="E0_smoke",
        condition="condition",
        fieldnames=("latency_ms",),
    ) as writer:
        writer.write({"latency_ms": 1.5}, timestamp_ns=123)

    text = path.read_text(encoding="utf-8")
    assert "timestamp_ns,run_id,experiment,condition,latency_ms" in text
    assert "123,run_001,E0_smoke,condition,1.5" in text


def test_clock_anchor_env_exports_shared_local_workload_clock():
    anchors = clock_anchor_env()

    assert int(anchors[CLOCK_WALL_ANCHOR_ENV]) > 0
    assert int(anchors[CLOCK_MONOTONIC_ANCHOR_ENV]) > 0


def test_numeric_summary_groups_heterogeneous_rows(tmp_path: Path):
    # Rows with different (category, name) must NOT be pooled into one statistic: a tf row with
    # duration_ms=0 must not drown a camera decode time (the cited p50=0.0 bug).
    path = tmp_path / "headset_metrics.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["timestamp_ns", "category", "name", "duration_ms"])
        writer.writeheader()
        for ts in (1, 2, 3, 4):
            writer.writerow({"timestamp_ns": ts, "category": "tf", "name": "received", "duration_ms": "0"})
        for ts, ms in ((5, "4.0"), (6, "5.0"), (7, "6.0")):
            writer.writerow({"timestamp_ns": ts, "category": "camera", "name": "displayed", "duration_ms": ms})

    summary = summarize_numeric_csv(path)
    assert "duration_ms" not in summary  # no misleading pooled stat
    assert "by_group" in summary
    camera = summary["by_group"]["camera|displayed"]["duration_ms"]
    assert camera["p50"] == 5.0  # real camera decode median, not 0.0


def test_aggregate_runs_reports_confidence_interval(tmp_path: Path):
    run_dirs = []
    for index, latency in enumerate((10.0, 12.0, 14.0, 11.0, 13.0)):
        run_dir = tmp_path / f"run_{index}"
        run_dir.mkdir()
        (run_dir / "derived_summary.json").write_text(
            json.dumps(
                {
                    "window_duration_s": 60.0,
                    "streams": {"camera|/c": {"latency_ms": {"p95": latency}, "drop_rate": 0.0}},
                    "bridge": {"dropped_count": 0},
                }
            ),
            encoding="utf-8",
        )
        run_dirs.append(run_dir)

    aggregate = aggregate_runs(run_dirs)
    assert aggregate["n_runs"] == 5
    metric = aggregate["metrics"]["streams.camera|/c.latency_ms.p95"]
    assert metric["n"] == 5
    assert 11.5 < metric["mean"] < 12.5
    assert metric["ci95_low"] < metric["mean"] < metric["ci95_high"]
    assert metric["ci95_half_width"] > 0.0


def test_numeric_summary_reports_percentiles(tmp_path: Path):
    path = tmp_path / "headset_metrics.csv"
    with CsvMetricWriter(
        path,
        run_id="run_001",
        experiment="E0_smoke",
        condition="condition",
        fieldnames=("latency_ms",),
    ) as writer:
        for value in (1.0, 2.0, 3.0):
            writer.write({"latency_ms": value})

    summary = summarize_numeric_csv(path)

    assert summary["row_count"] == 3
    assert summary["latency_ms"]["p50"] == 2.0


def test_paper_report_generates_tables_and_figures(tmp_path: Path):
    sdk_root = Path(__file__).resolve().parents[2]
    report_script = sdk_root / "python" / "examples" / "experiments" / "horus_paper_report.py"
    results_root = tmp_path / "results"
    run_dir = results_root / "e0_baseline_test_run"
    run_dir.mkdir(parents=True)

    (run_dir / "run_manifest.json").write_text(
        json.dumps(
            {
                "experiment": "E0_baseline",
                "condition": "one_robot_tf_low_rate_camera",
                "duration_s": 10,
                "warmup_s": 1,
                "robot_count": 1,
            }
        ),
        encoding="utf-8",
    )
    (run_dir / "data_quality.json").write_text(
        json.dumps({"ok": True, "measurement_valid": True, "within_envelope": True, "degraded": False}),
        encoding="utf-8",
    )
    (run_dir / "orchestrator_events.ndjson").write_text(
        "\n".join(
            [
                json.dumps({"timestamp_ns": 100, "name": "measurement_start"}),
                json.dumps({"timestamp_ns": 1000, "name": "measurement_end"}),
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    with (run_dir / "source_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_ns",
                "category",
                "name",
                "stream",
                "topic",
                "duration_ms",
                "value",
                "payload_bytes",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "timestamp_ns": 500,
                "category": "latency",
                "name": "tf_freshness",
                "stream": "tf",
                "topic": "/tf",
                "duration_ms": "12.5",
                "value": "12.5",
                "payload_bytes": "256",
            }
        )

    output_dir = tmp_path / "paper_report"
    subprocess.run(
        [
            sys.executable,
            str(report_script),
            "--results-root",
            str(results_root),
            "--output-dir",
            str(output_dir),
        ],
        check=True,
    )

    assert (output_dir / "paper_run_index.csv").exists()
    assert (output_dir / "paper_metric_summary.csv").exists()
    assert (output_dir / "paper_artifact_index.csv").exists()
    assert (output_dir / "tables" / "baseline_latency.csv").exists()
    assert (output_dir / "tables" / "baseline_latency.md").exists()
    assert (output_dir / "figures" / "baseline_latency.svg").exists()

    artifact_index = (output_dir / "paper_artifact_index.csv").read_text(encoding="utf-8")
    assert "baseline_latency,1" in artifact_index

    run_index = (output_dir / "paper_run_index.csv").read_text(encoding="utf-8")
    assert "measurement_valid" in run_index
    assert "within_envelope" in run_index
    assert "degraded" in run_index


def test_core_platform_experiments_stay_on_ros_bridge_path():
    sdk_root = Path(__file__).resolve().parents[2]
    configs_root = sdk_root / "python" / "examples" / "experiments" / "configs"
    core_configs = [
        "e6_control_under_sensor_load.json",
        "e7_multi_robot_scaling.json",
        "e9_failure_recovery.json",
    ]

    for config_name in core_configs:
        payload = json.loads((configs_root / config_name).read_text(encoding="utf-8"))
        camera = payload.get("camera") or {}
        assert "webrtc" not in str(payload.get("transport", "")).lower()
        assert str(camera.get("encoding", "")).lower() != "h264"


def test_registration_readiness_requires_source_tf_metric(tmp_path: Path):
    metrics_path = tmp_path / "source_metrics.csv"
    with metrics_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["timestamp_ns", "stream", "topic"])
        writer.writeheader()
        writer.writerow({"timestamp_ns": "1", "stream": "camera", "topic": "/camera/image_raw"})

    assert not source_metric_stream_seen(metrics_path, "tf")

    with metrics_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["timestamp_ns", "stream", "topic"])
        writer.writerow({"timestamp_ns": "2", "stream": "tf", "topic": "/tf"})

    assert source_metric_stream_seen(metrics_path, "tf")


def test_analysis_requires_clock_sync_and_derives_camera_latency(tmp_path: Path):
    run_dir = tmp_path / "e0_clocked_run"
    run_dir.mkdir()
    (run_dir / "run_manifest.json").write_text(
        json.dumps(
            {
                "experiment": "E0_baseline",
                "condition": "camera_latency",
                "duration_s": 0.3,
                "warmup_s": 0,
                "extra": {"workload": {"camera": {"streams": 1}}},
            }
        ),
        encoding="utf-8",
    )
    (run_dir / "orchestrator_events.ndjson").write_text(
        "\n".join(
            [
                json.dumps({"timestamp_ns": 1_000_000_000, "name": "measurement_start"}),
                json.dumps({"timestamp_ns": 1_300_000_000, "name": "measurement_end"}),
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (run_dir / "headset_events.ndjson").write_text(
        "\n".join(
            [
                json.dumps({"timestamp_ns": 1_100_000_000, "name": "measurement_start"}),
                json.dumps({"timestamp_ns": 1_400_000_000, "name": "measurement_end"}),
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (run_dir / "clock_sync.json").write_text(
        json.dumps(
            {
                "samples": [
                    {
                        "host_send_ns": 900_000_000,
                        "host_receive_ns": 910_000_000,
                        "headset_receive_ns": 1_005_000_000,
                        "headset_send_ns": 1_005_100_000,
                        "round_trip_ns": 9_900_000,
                        "offset_ns": 100_050_000,
                    },
                    {
                        "host_send_ns": 1_300_000_000,
                        "host_receive_ns": 1_310_000_000,
                        "headset_receive_ns": 1_405_000_000,
                        "headset_send_ns": 1_405_100_000,
                        "round_trip_ns": 9_900_000,
                        "offset_ns": 100_050_000,
                    },
                ],
                "summary": {
                    "sample_count": 2,
                    "offset_drift_ns": 0,
                    "round_trip_ns_max": 9_900_000,
                },
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    with CsvMetricWriter(
        run_dir / "source_metrics.csv",
        run_id=run_dir.name,
        experiment="E0_baseline",
        condition="camera_latency",
        fieldnames=("stream", "topic", "seq", "payload_bytes"),
    ) as writer:
        writer.write(
            {
                "stream": "camera",
                "topic": "/exp_robot_0/camera_0/image_raw/compressed",
                "seq": 1,
                "payload_bytes": 100,
            },
            timestamp_ns=1_100_000_000,
        )

    with CsvMetricWriter(
        run_dir / "headset_metrics.csv",
        run_id=run_dir.name,
        experiment="E0_baseline",
        condition="camera_latency",
        fieldnames=(
            "monotonic_ns",
            "source",
            "category",
            "name",
            "robot_id",
            "stream",
            "topic",
            "seq",
            "payload_bytes",
            "duration_ms",
            "value",
            "unit",
            "queue_depth",
            "frame_width",
            "frame_height",
            "points",
            "vertices",
            "triangles",
            "extra_json",
        ),
    ) as writer:
        writer.write(
            {
                "source": "horus_mr",
                "category": "frame",
                "name": "render",
                "stream": "render",
                "value": 72,
                "unit": "fps",
            },
            timestamp_ns=1_150_000_000,
        )
        writer.write(
            {
                "source": "horus_mr",
                "category": "camera",
                "name": "received_compressed",
                "stream": "left",
                "topic": "/exp_robot_0/camera_0/image_raw/compressed",
                "seq": 1,
                "payload_bytes": 100,
                "extra_json": json.dumps({"source_stamp_ns": 1_100_000_000}),
            },
            timestamp_ns=1_220_050_000,
        )

    write_summary(run_dir)
    quality = validate_run_directory(
        run_dir,
        {"source_metrics.csv": 1, "headset_metrics.csv": 1},
    )

    assert quality["ok"], quality
    assert quality["measurement_valid"], quality
    assert "within_envelope" in quality
    assert "degraded" in quality
    derived = list(csv.DictReader((run_dir / "derived_metrics.csv").open(encoding="utf-8")))
    assert len(derived) == 1
    assert derived[0]["name"] == "camera_received_compressed"
    assert 19.0 <= float(derived[0]["latency_ms"]) <= 21.0


_HEADSET_FIELDS = (
    "monotonic_ns", "source", "category", "name", "robot_id", "stream", "topic", "seq",
    "payload_bytes", "duration_ms", "value", "unit", "queue_depth", "frame_width", "frame_height",
    "points", "vertices", "triangles", "extra_json",
)


def _write_clock_scaffold(run_dir: Path, *, source_clock_synchronized: bool = True) -> None:
    """Manifest + measurement-window events + a 2-anchor clock_sync (offset ~100ms, drift 0)."""
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "run_manifest.json").write_text(
        json.dumps(
            {
                "experiment": "E11_map_pointcloud_sweep",
                "condition": "pointcloud_map_250k_5hz_lan",
                "duration_s": 0.3,
                "warmup_s": 0,
                "target_fps": 72,
                "source_clock": {"synchronized": source_clock_synchronized},
                "extra": {"workload": {"camera": {"streams": 1}, "pointcloud": {"points": 250000, "hz": 5}}},
            }
        ),
        encoding="utf-8",
    )
    for name in ("orchestrator_events.ndjson", "headset_events.ndjson"):
        start = 1_000_000_000 if name.startswith("orch") else 1_100_000_000
        end = 1_300_000_000 if name.startswith("orch") else 1_400_000_000
        (run_dir / name).write_text(
            json.dumps({"timestamp_ns": start, "name": "measurement_start"}) + "\n"
            + json.dumps({"timestamp_ns": end, "name": "measurement_end"}) + "\n",
            encoding="utf-8",
        )
    (run_dir / "clock_sync.json").write_text(
        json.dumps(
            {
                "samples": [
                    {"host_send_ns": 900_000_000, "host_receive_ns": 910_000_000,
                     "headset_receive_ns": 1_005_000_000, "headset_send_ns": 1_005_100_000,
                     "round_trip_ns": 9_900_000, "offset_ns": 100_050_000},
                    {"host_send_ns": 1_300_000_000, "host_receive_ns": 1_310_000_000,
                     "headset_receive_ns": 1_405_000_000, "headset_send_ns": 1_405_100_000,
                     "round_trip_ns": 9_900_000, "offset_ns": 100_050_000},
                ],
                "summary": {"sample_count": 2, "offset_drift_ns": 0,
                            "round_trip_ns_max": 9_900_000, "round_trip_ns_min": 9_900_000},
            },
            indent=2,
        ),
        encoding="utf-8",
    )


def _write_minimal_headset_frame(run_dir: Path, *, experiment: str = "E6_control_under_sensor_load") -> None:
    with CsvMetricWriter(
        run_dir / "headset_metrics.csv",
        run_id=run_dir.name,
        experiment=experiment,
        condition="control",
        fieldnames=_HEADSET_FIELDS,
    ) as writer:
        writer.write(
            {"source": "horus_mr", "category": "frame", "name": "render",
             "stream": "render", "value": 72, "unit": "fps"},
            timestamp_ns=1_200_000_000,
        )


def test_e6_derives_headset_command_to_bridge_publish_latency(tmp_path: Path):
    run_dir = tmp_path / "e6_control_latency"
    _write_clock_scaffold(run_dir)
    manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8"))
    manifest["experiment"] = "E6_control_under_sensor_load"
    manifest["condition"] = "camera_mesh_goal_load_lan"
    manifest["extra"]["workload"]["camera"] = {"streams": 0}
    manifest["extra"]["workload"]["pointcloud"] = {"points": 0, "hz": 0}
    manifest["extra"]["workload"]["extra"] = {"extra": {"control_load": True}}
    (run_dir / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    _write_minimal_headset_frame(run_dir)

    topic = "/exp_robot_0/cmd_vel"
    with CsvMetricWriter(
        run_dir / "headset_command_metrics.csv",
        run_id=run_dir.name,
        experiment="E6_control_under_sensor_load",
        condition="control",
        fieldnames=_HEADSET_FIELDS,
    ) as writer:
        # Wall-clock correction places this at host 1.120s.
        writer.write(
            {"source": "horus_mr", "category": "teleop", "name": "cmd_vel_published",
             "stream": "minimap", "topic": topic, "value": 0.4, "unit": "linear_speed"},
            timestamp_ns=1_220_050_000,
        )

    with (run_dir / "bridge_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "timestamp_ns", "run_id", "experiment", "condition", "source", "category",
                "name", "client_fd", "destination", "payload_bytes", "serialized_bytes",
                "queue_depth", "extra_json",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "timestamp_ns": 1_145_000_000,
                "run_id": run_dir.name,
                "experiment": "E6_control_under_sensor_load",
                "condition": "control",
                "source": "horus_ros2_bridge",
                "category": "unity_to_ros",
                "name": "published",
                "client_fd": -1,
                "destination": topic,
                "payload_bytes": 52,
                "serialized_bytes": 52,
                "queue_depth": 0,
                "extra_json": "",
            }
        )

    quality = validate_run_directory(
        run_dir,
        {"headset_metrics.csv": 1, "headset_command_metrics.csv": 1, "bridge_metrics.csv": 1},
    )

    assert quality["measurement_valid"], quality
    derived = list(csv.DictReader((run_dir / "derived_metrics.csv").open(encoding="utf-8")))
    command_rows = [row for row in derived if row["category"] == "command_latency"]
    assert len(command_rows) == 1
    assert command_rows[0]["topic"] == topic
    assert 24.0 <= float(command_rows[0]["latency_ms"]) <= 26.0
    summary = json.loads((run_dir / "derived_summary.json").read_text(encoding="utf-8"))
    command = summary["commands"][topic]
    assert command["headset_count"] == 1
    assert command["bridge_publish_count"] == 1
    assert command["matched_count"] == 1
    assert 24.0 <= command["headset_to_bridge_publish_ms"]["p50"] <= 26.0


def test_e6_rejects_missing_control_capture(tmp_path: Path):
    run_dir = tmp_path / "e6_missing_control"
    _write_clock_scaffold(run_dir)
    manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8"))
    manifest["experiment"] = "E6_control_under_sensor_load"
    manifest["condition"] = "camera_mesh_goal_load_lan"
    manifest["extra"]["workload"]["camera"] = {"streams": 0}
    manifest["extra"]["workload"]["pointcloud"] = {"points": 0, "hz": 0}
    manifest["extra"]["workload"]["extra"] = {"extra": {"control_load": True}}
    (run_dir / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    _write_minimal_headset_frame(run_dir)

    with (run_dir / "bridge_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["timestamp_ns", "category", "name", "destination"])
        writer.writeheader()

    quality = validate_run_directory(
        run_dir,
        {"headset_metrics.csv": 1, "bridge_metrics.csv": 0},
    )

    assert not quality["measurement_valid"], quality
    joined_errors = "\n".join(quality["errors"])
    assert "headset_command_metrics.csv did not contain command samples" in joined_errors


def test_derived_summary_drop_rate_uses_displayed_not_received(tmp_path: Path):
    # M1/M2: 4 frames sent; all 4 arrive at the headset (received) but only 2 survive the
    # coalesce-to-latest drain (displayed). drop_rate must be 0.5 (displayed/sent), NOT 0.0
    # (received/sent), and the glass-to-glass latency must not be pooled with arrival latency.
    # Source and headset seqs are DELIBERATELY mismatched to prove the join is on source_stamp_ns.
    run_dir = tmp_path / "e11_drop_run"
    _write_clock_scaffold(run_dir)

    topic = "/exp_robot_0/camera_0/image_raw/compressed"
    with CsvMetricWriter(
        run_dir / "source_metrics.csv", run_id=run_dir.name, experiment="E11_map_pointcloud_sweep",
        condition="c", fieldnames=("stream", "topic", "seq", "payload_bytes"),
    ) as writer:
        for seq in (1, 2, 3, 4):
            writer.write({"stream": "camera", "topic": topic, "seq": seq, "payload_bytes": 100},
                         timestamp_ns=1_100_000_000)

    with CsvMetricWriter(
        run_dir / "headset_metrics.csv", run_id=run_dir.name, experiment="E11_map_pointcloud_sweep",
        condition="c", fieldnames=_HEADSET_FIELDS,
    ) as writer:
        # all four arrive (received) at headset_ts -> corrected 1.120e9 -> 20 ms arrival latency
        for hseq in (101, 102, 103, 104):
            writer.write(
                {"source": "horus_mr", "category": "camera", "name": "received_compressed",
                 "stream": "left", "topic": topic, "seq": hseq, "payload_bytes": 100,
                 "extra_json": json.dumps({"source_stamp_ns": 1_100_000_000})},
                timestamp_ns=1_220_050_000,
            )
        # only two are displayed at headset_ts -> corrected 1.150e9 -> 50 ms glass-to-glass latency
        for hseq in (201, 202):
            writer.write(
                {"source": "horus_mr", "category": "camera", "name": "displayed_compressed",
                 "stream": "left", "topic": topic, "seq": hseq, "payload_bytes": 100,
                 "extra_json": json.dumps({"source_stamp_ns": 1_100_000_000})},
                timestamp_ns=1_250_050_000,
            )

    write_summary(run_dir)
    summary = json.loads((run_dir / "derived_summary.json").read_text(encoding="utf-8"))
    stream = summary["streams"]["camera|" + topic]

    assert stream["sent_count"] == 4
    assert stream["received_count"] == 4
    assert stream["displayed_count"] == 2
    assert abs(stream["delivery_rate"] - 1.0) < 1e-9       # all arrived
    assert abs(stream["display_rate"] - 0.5) < 1e-9        # only half shown
    assert abs(stream["drop_rate"] - 0.5) < 1e-9           # headline drop = displayed-based
    assert abs(stream["headset_coalesce_rate"] - 0.5) < 1e-9
    # latency must be split: displayed (glass-to-glass ~50ms) separate from arrival (~20ms)
    assert 49.0 <= stream["latency_ms"]["p50"] <= 51.0
    assert 19.0 <= stream["arrival_latency_ms"]["p50"] <= 21.0
    assert stream["latency_source_clock_synchronized"] is True
    assert summary["source_clock_synchronized"] is True


def test_derived_summary_refuses_topic_seq_latency_join_without_source_stamp(tmp_path: Path):
    run_dir = tmp_path / "e11_no_source_stamp"
    _write_clock_scaffold(run_dir)

    topic = "/exp_robot_0/camera_0/image_raw/compressed"
    with CsvMetricWriter(
        run_dir / "source_metrics.csv", run_id=run_dir.name, experiment="E11_map_pointcloud_sweep",
        condition="c", fieldnames=("stream", "topic", "seq", "payload_bytes"),
    ) as writer:
        writer.write({"stream": "camera", "topic": topic, "seq": 1, "payload_bytes": 100},
                     timestamp_ns=1_100_000_000)

    with CsvMetricWriter(
        run_dir / "headset_metrics.csv", run_id=run_dir.name, experiment="E11_map_pointcloud_sweep",
        condition="c", fieldnames=_HEADSET_FIELDS,
    ) as writer:
        writer.write(
            {"source": "horus_mr", "category": "camera", "name": "displayed_compressed",
             "stream": "left", "topic": topic, "seq": 1, "payload_bytes": 100,
             "extra_json": "{}"},
            timestamp_ns=1_250_050_000,
        )

    write_summary(run_dir)
    derived_rows = list(csv.DictReader((run_dir / "derived_metrics.csv").open(encoding="utf-8")))
    assert derived_rows == []
    summary = json.loads((run_dir / "derived_summary.json").read_text(encoding="utf-8"))
    stream = summary["streams"]["camera|" + topic]
    assert stream["displayed_count"] == 1
    assert "latency_ms" not in stream


def test_derived_summary_marks_fully_starved_stream_as_full_drop(tmp_path: Path):
    # Survivorship: a camera stream sent by the source with ZERO headset rows is fully starved and
    # must read drop_rate=1.0, not vanish from the summary (else aggregation hides the worst run).
    run_dir = tmp_path / "e11_starved_run"
    _write_clock_scaffold(run_dir)
    topic = "/exp_robot_0/camera_0/image_raw/compressed"
    with CsvMetricWriter(
        run_dir / "source_metrics.csv", run_id=run_dir.name, experiment="E11_map_pointcloud_sweep",
        condition="c", fieldnames=("stream", "topic", "seq", "payload_bytes"),
    ) as writer:
        for seq in (1, 2, 3):
            writer.write({"stream": "camera", "topic": topic, "seq": seq, "payload_bytes": 100},
                         timestamp_ns=1_100_000_000)
    # a headset file must exist (frame rows only) so the run is otherwise valid
    with CsvMetricWriter(
        run_dir / "headset_metrics.csv", run_id=run_dir.name, experiment="E11_map_pointcloud_sweep",
        condition="c", fieldnames=_HEADSET_FIELDS,
    ) as writer:
        writer.write({"source": "horus_mr", "category": "frame", "name": "render",
                      "stream": "render", "value": 30, "unit": "fps"}, timestamp_ns=1_200_000_000)

    write_summary(run_dir)
    summary = json.loads((run_dir / "derived_summary.json").read_text(encoding="utf-8"))
    stream = summary["streams"]["camera|" + topic]
    assert stream["displayed_count"] == 0
    assert abs(stream["drop_rate"] - 1.0) < 1e-9


def test_analysis_rejects_static_pointcloud_capacity_payload(tmp_path: Path):
    run_dir = tmp_path / "e3_static_pointcloud"
    _write_clock_scaffold(run_dir)
    manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8"))
    manifest["experiment"] = "E3_pointcloud_capacity"
    manifest["condition"] = "250k_points_5hz_1robot_lan"
    manifest["robot_count"] = 1
    manifest["extra"]["workload"]["camera"] = {"streams": 0}
    manifest["extra"]["workload"]["pointcloud"] = {"points": 250000, "hz": 5}
    (run_dir / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    topic = "/exp_robot_0/points"
    with CsvMetricWriter(
        run_dir / "source_metrics.csv",
        run_id=run_dir.name,
        experiment="E3_pointcloud_capacity",
        condition="c",
        fieldnames=("stream", "topic", "seq", "payload_bytes", "points", "notes"),
    ) as writer:
        for seq in (1, 2, 3, 4):
            writer.write(
                {
                    "stream": "pointcloud",
                    "topic": topic,
                    "seq": seq,
                    "payload_bytes": 4_000_000,
                    "points": 250000,
                    "notes": "",
                },
                timestamp_ns=1_100_000_000 + seq,
            )

    with CsvMetricWriter(
        run_dir / "headset_metrics.csv",
        run_id=run_dir.name,
        experiment="E3_pointcloud_capacity",
        condition="c",
        fieldnames=_HEADSET_FIELDS,
    ) as writer:
        writer.write(
            {"source": "horus_mr", "category": "frame", "name": "render",
             "stream": "render", "value": 72, "unit": "fps"},
            timestamp_ns=1_200_000_000,
        )
        writer.write(
            {"source": "horus_mr", "category": "pointcloud", "name": "uploaded",
             "stream": "pointcloud", "topic": topic, "payload_bytes": 4_000_000,
             "points": 250000, "extra_json": json.dumps({"skipped_identical_bytes": False})},
            timestamp_ns=1_210_000_000,
        )
        for seq in (1, 2, 3):
            writer.write(
                {"source": "horus_mr", "category": "pointcloud", "name": "received",
                 "stream": "pointcloud", "topic": topic, "payload_bytes": 4_000_000,
                 "points": 250000, "extra_json": json.dumps({"raw_sequence": seq})},
                timestamp_ns=1_220_000_000 + seq,
            )
            writer.write(
                {"source": "horus_mr", "category": "pointcloud", "name": "skipped",
                 "stream": "pointcloud", "topic": topic, "payload_bytes": 4_000_000,
                 "points": 250000, "extra_json": json.dumps({"skipped_identical_bytes": True})},
                timestamp_ns=1_230_000_000 + seq,
            )

    quality = validate_run_directory(
        run_dir,
        {"source_metrics.csv": 1, "headset_metrics.csv": 1},
    )

    assert not quality["measurement_valid"], quality
    joined_errors = "\n".join(quality["errors"])
    assert "source pointcloud topic did not show animated payload variation" in joined_errors
    assert "identical-payload skips" in joined_errors


def test_analysis_rejects_mesh_map_without_headset_delivery(tmp_path: Path):
    run_dir = tmp_path / "e4_mesh_missing_headset"
    _write_clock_scaffold(run_dir)
    manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8"))
    manifest["experiment"] = "E4_mesh_vs_pointcloud_map"
    manifest["condition"] = "triangle_shell_mesh_24k_triangles_lan"
    manifest["extra"]["workload"]["camera"] = {"streams": 0}
    manifest["extra"]["workload"]["pointcloud"] = {"points": 0, "hz": 0}
    manifest["extra"]["workload"]["map"] = {
        "representation": "triangle_shell_mesh",
        "chunks": 8,
        "triangles": 24000,
        "vertices": 16000,
        "hz": 1,
    }
    (run_dir / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    with CsvMetricWriter(
        run_dir / "source_metrics.csv",
        run_id=run_dir.name,
        experiment="E4_mesh_vs_pointcloud_map",
        condition="c",
        fieldnames=("stream", "topic", "seq", "payload_bytes", "vertices", "triangles"),
    ) as writer:
        writer.write(
            {
                "stream": "mesh",
                "topic": "/horus/experiment/map_mesh",
                "seq": 1,
                "payload_bytes": 192_000,
                "vertices": 16000,
                "triangles": 24000,
            },
            timestamp_ns=1_100_000_000,
        )

    with CsvMetricWriter(
        run_dir / "headset_metrics.csv",
        run_id=run_dir.name,
        experiment="E4_mesh_vs_pointcloud_map",
        condition="c",
        fieldnames=_HEADSET_FIELDS,
    ) as writer:
        writer.write(
            {"source": "horus_mr", "category": "frame", "name": "render",
             "stream": "render", "value": 72, "unit": "fps"},
            timestamp_ns=1_200_000_000,
        )

    quality = validate_run_directory(
        run_dir,
        {"source_metrics.csv": 1, "headset_metrics.csv": 1},
    )

    assert not quality["measurement_valid"], quality
    joined_errors = "\n".join(quality["errors"])
    assert "mesh_map marker_received" in joined_errors
    assert "mesh_map finalized" in joined_errors


def test_analysis_accepts_mesh_map_with_headset_delivery(tmp_path: Path):
    run_dir = tmp_path / "e4_mesh_with_headset"
    _write_clock_scaffold(run_dir)
    manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8"))
    manifest["experiment"] = "E4_mesh_vs_pointcloud_map"
    manifest["condition"] = "triangle_shell_mesh_24k_triangles_lan"
    manifest["extra"]["workload"]["camera"] = {"streams": 0}
    manifest["extra"]["workload"]["pointcloud"] = {"points": 0, "hz": 0}
    manifest["extra"]["workload"]["map"] = {
        "representation": "triangle_shell_mesh",
        "chunks": 8,
        "triangles": 24000,
        "vertices": 16000,
        "hz": 1,
    }
    (run_dir / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    topic = "/horus/experiment/map_mesh"
    with CsvMetricWriter(
        run_dir / "source_metrics.csv",
        run_id=run_dir.name,
        experiment="E4_mesh_vs_pointcloud_map",
        condition="c",
        fieldnames=("stream", "topic", "seq", "payload_bytes", "vertices", "triangles"),
    ) as writer:
        writer.write(
            {
                "stream": "mesh",
                "topic": topic,
                "seq": 1,
                "payload_bytes": 192_000,
                "vertices": 16000,
                "triangles": 24000,
            },
            timestamp_ns=1_100_000_000,
        )

    with CsvMetricWriter(
        run_dir / "headset_metrics.csv",
        run_id=run_dir.name,
        experiment="E4_mesh_vs_pointcloud_map",
        condition="c",
        fieldnames=_HEADSET_FIELDS,
    ) as writer:
        writer.write(
            {"source": "horus_mr", "category": "frame", "name": "render",
             "stream": "render", "value": 72, "unit": "fps"},
            timestamp_ns=1_200_000_000,
        )
        writer.write(
            {"source": "horus_mr", "category": "mesh_map", "name": "marker_received",
             "stream": "marker", "topic": topic, "seq": 1, "points": 9000, "triangles": 3000},
            timestamp_ns=1_210_000_000,
        )
        writer.write(
            {"source": "horus_mr", "category": "mesh_map", "name": "finalized",
             "stream": "marker", "topic": topic, "value": 8, "unit": "active_chunks",
             "vertices": 16000, "triangles": 24000},
            timestamp_ns=1_220_000_000,
        )

    quality = validate_run_directory(
        run_dir,
        {"source_metrics.csv": 1, "headset_metrics.csv": 1},
    )

    assert quality["measurement_valid"], quality


def test_find_saturation_knee_locates_sustained_degradation():
    conds = [
        {"label": "100k", "payload": 100000, "payload_kind": "points", "source_hz": 5,
         "achieved_hz": 5.0, "drop_rate": 0.0, "latency_p95_ms": 40, "headset_fps": 72, "target_fps": 72},
        {"label": "250k", "payload": 250000, "payload_kind": "points", "source_hz": 5,
         "achieved_hz": 4.9, "drop_rate": 0.01, "latency_p95_ms": 55, "headset_fps": 71, "target_fps": 72},
        {"label": "500k", "payload": 500000, "payload_kind": "points", "source_hz": 5,
         "achieved_hz": 4.2, "drop_rate": 0.08, "latency_p95_ms": 95, "headset_fps": 60, "target_fps": 72},
        {"label": "1000k", "payload": 1000000, "payload_kind": "points", "source_hz": 5,
         "achieved_hz": 3.1, "drop_rate": 0.30, "latency_p95_ms": 210, "headset_fps": 44, "target_fps": 72},
    ]
    result = find_saturation_knee(conds)
    assert result["knee"] == "500k"
    assert result["knee_payload"] == 500000


def test_find_saturation_knee_ignores_transient_blip():
    # A single degraded point that recovers is NOT the knee (must be sustained).
    conds = [
        {"label": "a", "payload": 1, "source_hz": 5, "achieved_hz": 5.0},
        {"label": "b", "payload": 2, "source_hz": 5, "achieved_hz": 4.0},  # blip
        {"label": "c", "payload": 3, "source_hz": 5, "achieved_hz": 5.0},
        {"label": "d", "payload": 4, "source_hz": 5, "achieved_hz": 5.0},
    ]
    assert find_saturation_knee(conds)["knee"] is None


def test_aggregate_runs_surfaces_metric_missing_in_some_runs(tmp_path: Path):
    # One run reports drop_rate, another does not -> coverage must flag the gap, not silently
    # average over the runs that happened to have it.
    run_dirs = []
    for index, payload in enumerate(
        [
            {"streams": {"camera|/c": {"latency_ms": {"p95": 10.0}, "drop_rate": 0.0}}},
            {"streams": {"camera|/c": {"latency_ms": {"p95": 12.0}}}},  # no drop_rate
        ]
    ):
        run_dir = tmp_path / f"run_{index}"
        run_dir.mkdir()
        (run_dir / "derived_summary.json").write_text(json.dumps(payload), encoding="utf-8")
        run_dirs.append(run_dir)

    aggregate = aggregate_runs(run_dirs)
    assert aggregate["n_runs"] == 2
    drop_key = "streams.camera|/c.drop_rate"
    assert aggregate["metrics"][drop_key]["n"] == 1
    assert abs(aggregate["metrics"][drop_key]["coverage"] - 0.5) < 1e-9
    assert drop_key in aggregate["metrics_missing_in_some_runs"]
