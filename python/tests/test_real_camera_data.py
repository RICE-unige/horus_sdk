import importlib.util
import csv
import json
from pathlib import Path

import numpy as np
import pytest

PATH = Path(__file__).parents[1] / "examples/tools/real_camera_data.py"
spec = importlib.util.spec_from_file_location("real_camera_data", PATH)
data = importlib.util.module_from_spec(spec)
spec.loader.exec_module(data)


def test_timestamps_are_preserved_and_loop_has_one_frame_interval(tmp_path):
    path = tmp_path / "rgb.txt"
    path.write_text("# source stamps\n10.000 rgb/a.png\n10.033 rgb/b.png\n10.067 rgb/c.png\n")
    rows = data.read_index(path)
    assert rows[1] == (10.033, "rgb/b.png")
    assert data.loop_period([x[0] for x in rows]) == pytest.approx(.1005)
    path.write_text("1 a\n1 b\n")
    with pytest.raises(ValueError):
        data.read_index(path)


def test_depth_association_is_bounded_and_cloud_keeps_every_valid_pixel():
    assert data.nearest_index([1., 2.], 1.02) == 0
    assert data.nearest_index([1., 2.], 1.5) is None
    depth = np.array([[5000, 0], [10000, 15000]], dtype=np.uint16)
    rgb = np.array([[[255, 2, 3], [0, 0, 0]], [[4, 255, 6], [7, 8, 255]]], dtype=np.uint8)
    cloud = data.pointcloud(depth, rgb)
    assert len(cloud) == 3
    assert cloud["z"].tolist() == [1., 2., 3.]
    assert cloud["rgb"].tolist() == [0xff0203, 0x04ff06, 0x0708ff]
    assert cloud.dtype.itemsize == 16


def test_clock_estimate_exposes_uncertainty_and_expires():
    clock = data.ClockEstimate()
    assert clock.add(1000, 1100, 1520, 1530)
    sample = clock.best(1200)
    assert sample[1:] == (475., 45.)
    assert not clock.add(1100, 1000, 1520, 1530)
    assert clock.best(16_000_000_000) is None


def test_rate_and_summary_include_the_real_window():
    rate = data.Rate(seconds=1)
    for time in (1., 1.1, 1.2):
        rate.tick(time)
    assert rate.hz(1.2) == pytest.approx(10)
    assert rate.hz(3.) == 0
    assert data.summary([1., 1.1, 1.2])["fps"] == pytest.approx(10)


def test_analysis_matches_unique_source_identities_and_clock_bounds(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(PATH.parent))
    spec = importlib.util.spec_from_file_location("analyze_real_camera_experiment", PATH.with_name("analyze_real_camera_experiment.py"))
    analysis = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analysis)
    (tmp_path / "result.json").write_text(json.dumps(dict(condition="video_only", cameras=1,
        measurement_start_wall_ns=0, measurement_end_wall_ns=10_000_000_000,
        duration_s=10, decoded={"cam1": {}}, inbound_stats=[])))
    (tmp_path / "clock-samples.json").write_text(json.dumps([dict(sent_ns=0, received_ns=2_000_000,
        source_received_ns=1_500_000, source_sent_ns=1_500_000)]))
    (tmp_path / "source-traces.json").write_text(json.dumps({"cam1": [
        dict(receipt_wall_ns=int(t*1e9)+10_000_000, ros_stamp_ns=int(t*1e9)+500_000)
        for t in (4, 6)]}))
    columns = ["kind", "stream", "publish_wall_ns", "publish_monotonic_ns", "recorded_timestamp", "cycle", "index", "bytes"]
    source = tmp_path / "source.csv"
    with source.open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for t in (4, 5, 6):
            writer.writerow(["rgb", 1, int(t*1e9)+500_000, int(t*1e9), 100+t, 0, t, 921600])
    (tmp_path / "received_events.csv").write_text("kind,stream,receipt_wall_ns,receipt_monotonic_ns,source_publish_wall_ns,bytes\n")
    result = analysis.analyze(tmp_path, [source])
    assert result["webrtc_decoder"] == "aiortc"
    assert "excludes transport headers" in result["payload_measurement"]
    value = result["cameras"]["cam1"]
    assert value["eligible_source_frames"] == 3
    assert value["source_identity_delivery_percent"] == pytest.approx(200/3)
    assert value["replay_publish_to_decode_ms"]["p50"] == 10
    assert value["max_clock_uncertainty_ms"] == 1


def test_depth_png_validation_rejects_bad_data_without_substitution():
    cv2 = pytest.importorskip("cv2")
    original = np.full((480,640), 5000, dtype=np.uint16)
    ok, encoded = cv2.imencode(".png", original)
    assert ok
    np.testing.assert_array_equal(data.decode_depth_png(encoded.tobytes()), original)
    with pytest.raises(ValueError):
        data.decode_depth_png(b"not a png")
    ok, wrong = cv2.imencode(".png", np.zeros((2,2), dtype=np.uint16))
    with pytest.raises(ValueError):
        data.decode_depth_png(wrong.tobytes())


def test_window_metrics_count_leading_trailing_and_total_outages():
    result = data.window_summary([4., 4.1, 4.2], 0., 10.)
    assert result["window_fps"] == pytest.approx(.3)
    assert result["max_silence_ms"] == pytest.approx(5800)
    assert result["gaps_over_1s"] == 2
    assert data.window_summary([], 0., 10.)["max_silence_ms"] == 10000
    with pytest.raises(ValueError):
        data.window_summary([], 1., 1.)


def test_integrity_matches_source_identity_not_merely_valid_image_headers():
    source = [dict(kind="jpeg", stream="1", publish_wall_ns="10", sha256="abc")]
    received = [dict(kind="jpeg", stream="cam1", source_publish_wall_ns="10", sha256="abc"),
                dict(kind="jpeg", stream="cam1", source_publish_wall_ns="10", sha256="bad")]
    result = data.payload_integrity(source, received)
    assert result["checked"] == 2
    assert result["mismatches"] == [dict(kind="jpeg", stream="1", stamp="10")]


def test_quality_uses_exact_frame_identity_and_reports_unmatched(tmp_path):
    cv2 = pytest.importorskip("cv2")
    import sys
    sys.modules["real_camera_data"] = data
    spec = importlib.util.spec_from_file_location("real_camera_quality", PATH.with_name("real_camera_quality.py"))
    quality = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(quality)
    dataset = tmp_path / "dataset"
    dataset.mkdir()
    original = np.full((4,5,3),128,np.uint8)
    cv2.imwrite(str(dataset/"a.png"),original)
    cv2.imwrite(str(dataset/"b.png"),np.zeros_like(original))
    (dataset/"rgb.txt").write_text("1 a.png\n2 b.png\n")
    samples = tmp_path / "trial/quality"
    samples.mkdir(parents=True)
    cv2.imwrite(str(samples/"cam1-100.png"),original)
    cv2.imwrite(str(samples/"cam1-999.png"),original)
    events = tmp_path / "source.csv"
    events.write_text("kind,stream,publish_wall_ns,index\nrgb,1,100,0\n")
    result = quality.quality(tmp_path,[events],dataset)
    assert result["samples"][0]["identical"] is True
    assert result["samples"][0]["recorded_index"] == 0
    assert result["unmatched_samples"] == ["trial/quality/cam1-999.png"]
