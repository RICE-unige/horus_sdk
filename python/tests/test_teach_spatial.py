import dataclasses
import pytest

from horus.teaching import TeachConfig, TeachPointCloud, TeachStream


def test_spatial_configuration_registration_and_roundtrip():
    stream = TeachStream("depth", "/robot/front/points", "sensor_msgs/msg/PointCloud2", required=False)
    config = TeachConfig("/tmp/episodes", streams=(stream,), spatial_views=(TeachPointCloud("depth"),))
    assert config.digest == "25c4fbfbee7b73fd598a923b7ac5e46b20527f3fa67fc9e8d85ec55c64627e91"
    assert TeachConfig.from_dict(config.to_dict()) == config
    assert config.registration("robot")["spatial_views"] == [dict(
        stream_id="depth", topic=stream.topic, point_size_m=.004, max_points=1000000, max_age_seconds=1.0)]
    assert "spatial_views" not in TeachConfig("/tmp/episodes").to_dict()
    assert "spatial_views" not in TeachConfig("/tmp/episodes").registration("robot")


@pytest.mark.parametrize("options", [dict(max_points=True), dict(max_points=999), dict(max_points=2000001),
    dict(point_size_m=float("nan")), dict(point_size_m=0), dict(max_age_seconds=0), dict(max_age_seconds=float("inf"))])
def test_spatial_configuration_bounds(options):
    with pytest.raises(ValueError):
        TeachPointCloud("depth", **options)


def test_spatial_view_requires_correct_stream_and_unique_binding():
    view = TeachPointCloud("depth")
    with pytest.raises(ValueError): TeachConfig("/tmp/episodes", spatial_views=(view,))
    wrong = TeachStream("depth", "/camera", "sensor_msgs/msg/Image")
    with pytest.raises(ValueError): TeachConfig("/tmp/episodes", streams=(wrong,), spatial_views=(view,))
    cloud = dataclasses.replace(wrong, message_type="sensor_msgs/msg/PointCloud2")
    with pytest.raises(ValueError): TeachConfig("/tmp/episodes", streams=(cloud,), spatial_views=(view, view))
