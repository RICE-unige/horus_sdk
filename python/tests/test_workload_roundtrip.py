import copy
import pytest
from horus.experiments.workloads import workload_from_dict


def test_workload_roundtrip_preserves_input_and_extra():
    payload = dict(robot_count=3, camera=dict(streams=2), pointcloud=dict(points=10, hz=2),
                   map=dict(representation="mesh", hz=1), seed=42, extra=dict(test=True))
    original = copy.deepcopy(payload)
    workload = workload_from_dict(payload)
    assert payload == original
    assert workload.stream_count == 8
    assert workload.extra == dict(test=True, seed=42)
    assert workload_from_dict(workload.to_dict()).to_dict() == workload.to_dict()


def test_extra_requires_object():
    with pytest.raises(ValueError, match="extra"):
        workload_from_dict(dict(extra=[]))
