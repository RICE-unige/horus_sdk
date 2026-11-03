import numpy as np
import pytest

from horus.mapping.costmap import CostmapState, GridGeometry, RAW_TO_OCCUPANCY
from horus.mapping.ros_costmap import arguments


def geometry(width=4, height=3, frame="map"):
    return GridGeometry(frame, width, height, 0.05, (1, 2, 0, 0, 0, 0, 1))


def test_every_raw_cost_matches_nav2_display_encoding():
    expected = [0] + [1 + (97 * (i - 1)) // 251 for i in range(1, 253)] + [99, 100, -1]
    assert RAW_TO_OCCUPANCY.tolist() == expected
    state = CostmapState()
    assert state.full(geometry(16, 16), list(range(256)), 1, raw=True)
    assert state.data.reshape(-1).tolist() == expected


def test_patches_retain_unmodified_cells_and_source_array_is_not_aliased():
    source = np.arange(12, dtype=np.int8)
    state = CostmapState()
    state.full(geometry(), source, 10)
    source[:] = 99
    state.patch("map", 1, 1, 2, 2, [100, -1, 99, 0], 11)
    assert state.data.tolist() == [[0, 1, 2, 3], [4, 100, -1, 7], [8, 99, 0, 11]]
    assert not state.patch("map", 0, 0, 1, 1, [50], 9)
    assert not state.full(geometry(), [50] * 12, 9)
    assert state.stamp_ns == 11


@pytest.mark.parametrize("patch", [
    ("wrong_frame", 0, 0, 1, 1, [1]),
    ("map", -1, 0, 1, 1, [1]),
    ("map", 3, 0, 2, 1, [1, 1]),
    ("map", 0, 0, 2, 1, [1]),
    ("map", 0, 0, 1, 1, [101]),
    ("map", 0, 0, 1, 1, [1.5]),
    ("map", True, 0, 1, 1, [1]),
])
def test_invalid_patch_cannot_partially_mutate_map(patch):
    state = CostmapState()
    state.full(geometry(), [0] * 12, 10)
    with pytest.raises(ValueError):
        state.patch(*patch, 11)
    assert not np.any(state.data)
    assert state.stamp_ns == 10


def test_rolling_window_full_map_replaces_old_geometry_and_cells():
    state = CostmapState()
    state.full(geometry(), [5] * 12, 10)
    replacement = GridGeometry("odom", 2, 1, 0.1, (9, 8, 0, 0, 0, 0, 1))
    state.full(replacement, [253, 255], 11, raw=True)
    assert state.geometry == replacement
    assert state.data.tolist() == [[99, -1]]
    with pytest.raises(ValueError):
        state.patch("map", 0, 0, 1, 1, [1], 12)


@pytest.mark.parametrize("value", [
    geometry(0, 1), geometry(4097, 1), geometry(4096, 4096),
    GridGeometry("", 1, 1, .05, (0, 0, 0, 0, 0, 0, 1)),
    GridGeometry("map", 1, 1, float("nan"), (0, 0, 0, 0, 0, 0, 1)),
    GridGeometry("map", 1, 1, .05, (0, 0, 0, 0, 0, 0, 0)),
    GridGeometry("map", 1, 1, .05, (0, 0, 0, 0, 0, 0, 2)),
    GridGeometry("map", 1, 1, 1e40, (0, 0, 0, 0, 0, 0, 1)),
    GridGeometry("map", 1, 1, .05, (1e40, 0, 0, 0, 0, 0, 1)),
])
def test_bad_geometry_rejected_before_allocation(value):
    with pytest.raises(ValueError):
        CostmapState().full(value, [], 0)


def test_incremental_cli_requires_snapshot_recovery_and_distinct_topics():
    with pytest.raises(SystemExit):
        arguments(["--input-topic", "/costmap", "--updates-topic", "/costmap_updates"])
    with pytest.raises(SystemExit):
        arguments(["--input-topic", "/costmap", "--output-topic", "/costmap"])
    args, rest = arguments(["--input-topic", "/costmap_raw", "--input-format", "raw",
        "--updates-topic", "/costmap_raw_updates", "--snapshot-service", "/get_costmap",
        "--ros-args", "-p", "use_sim_time:=true"])
    assert args.input_format == "raw"
    assert rest == ["--ros-args", "-p", "use_sim_time:=true"]
