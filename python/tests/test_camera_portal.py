from dataclasses import replace
from types import SimpleNamespace
import threading
import time

import numpy as np
import pytest

from horus.teaching import TeachCamera, TeachConfig, TeachPortal
from horus.teaching.camera_portal import LatestPreviewWorker, RelativeStereoWarp, image_rgb, inference_shape


def test_opt_in_contract_and_roundtrip():
    old = TeachConfig("/data/teach", cameras=(TeachCamera("front"),))
    assert "portal" not in old.to_dict()["cameras"][0]
    assert "portal" not in old.registration("robot")["cameras"][0]
    for mode in ("mono", "stereo", "estimated"):
        config = replace(old, cameras=(TeachCamera("front", portal=TeachPortal(mode=mode)),))
        assert TeachConfig.from_dict(config.to_dict()).digest == config.digest
        assert config.digest != old.digest
        assert config.registration("robot")["cameras"][0]["portal"]["mode"] == mode


@pytest.mark.parametrize("options", [{"mode": "fake"}, {"edge_fade": float("nan")},
    {"crop": -.1}, {"crop": .2}, {"max_skew_ms": 101}, {"swap_eyes": 1}, {"corner_radius": True}])
def test_invalid_portal(options):
    with pytest.raises(ValueError): TeachPortal(**options)


def test_depth_dependent_disparity_and_source_preserved():
    pytest.importorskip("cv2")
    rgb = np.zeros((40, 200, 3), np.uint8)
    rgb[:, :, 0] = np.arange(200)
    original = rgb.copy()
    depth = np.repeat(np.linspace(0, 1, 40)[:, None], 200, axis=1)
    pair = RelativeStereoWarp(.015).render(rgb, depth)
    assert pair.shape == (40, 400, 3)
    # Near geometry shifts right in the left eye and left in the right eye.
    assert pair[35, 100, 0] < rgb[35, 100, 0] < pair[35, 300, 0]
    assert pair[3, 100, 0] > rgb[3, 100, 0] > pair[3, 300, 0]
    np.testing.assert_array_equal(rgb, original)
    flat = RelativeStereoWarp().render(rgb, np.ones((4, 4)))
    np.testing.assert_array_equal(flat[:, :200], flat[:, 200:])
    with pytest.raises(ValueError): RelativeStereoWarp().render(rgb, np.full((4, 4), np.nan))


def test_raw_padding_and_bad_sizes():
    pytest.importorskip("cv2")
    pytest.importorskip("PIL")
    source = SimpleNamespace(width=2, height=1, step=8, encoding="bgr8", data=bytes([0, 0, 255, 0, 255, 0, 1, 2]))
    assert image_rgb(source).tolist() == [[[255, 0, 0], [0, 255, 0]]]
    source.data = b"short"
    with pytest.raises(ValueError): image_rgb(source)


@pytest.mark.parametrize("height,width", [(720, 1280), (8192, 1), (1, 8192), (240, 320)])
def test_inference_size_is_bounded_without_upscaling_long_axis(height, width):
    size = inference_shape(height, width, 294)
    assert max(size.values()) == 294
    assert min(size.values()) >= 14
    assert all(value % 14 == 0 for value in size.values())


def test_compressed_roundtrip_and_rejection():
    cv2 = pytest.importorskip("cv2")
    pytest.importorskip("PIL")
    _, data = cv2.imencode(".png", np.full((8, 10, 3), (0, 0, 255), np.uint8))
    result = image_rgb(SimpleNamespace(data=data.tobytes()), True)
    assert result.shape == (8, 10, 3) and result[0, 0].tolist() == [255, 0, 0]
    with pytest.raises(Exception): image_rgb(SimpleNamespace(data=b"not an image"), True)


def test_latest_frame_queue_and_shutdown():
    entered, unblock, done = threading.Event(), threading.Event(), threading.Event()
    seen, errors = [], []
    def process(value):
        if value == 0:
            entered.set()
            assert unblock.wait(2)
        return value
    def publish(source, result):
        seen.append(result)
        if result == 99: done.set()
    worker = LatestPreviewWorker(process, publish, errors.append, fps=30, max_age=2)
    try:
        worker.submit(0)
        assert entered.wait(2)
        for value in range(1, 100): worker.submit(value)
        unblock.set()
        assert done.wait(2)
        assert seen == [0, 99] and worker.replaced == 98
    finally:
        unblock.set()
        worker.close()
    assert not worker.thread.is_alive() and not errors


def test_expired_inference_does_not_publish():
    finished = threading.Event()
    output = []
    def process(value):
        time.sleep(.08)
        finished.set()
        return value
    worker = LatestPreviewWorker(process, lambda *args: output.append(args), lambda e: None, max_age=.05)
    worker.submit(1)
    assert finished.wait(2)
    time.sleep(.02)
    worker.close()
    assert not output and worker.expired == 1
