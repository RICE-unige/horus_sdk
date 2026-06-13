from pathlib import Path

from horus.experiments.analysis import summarize_numeric_csv
from horus.experiments.manifest import ExperimentManifest, RunIdentity
from horus.experiments.metrics import CsvMetricWriter


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
