"""CPU-only colour/code-point and patched-adapter regressions."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

from horus.remote_rendering.nvenc_encoder import GpuH264Encoder, _limited_code_to_rgb


def test_auxiliary_lookup_preserves_all_valid_limited_code_points():
    lookup = _limited_code_to_rgb()
    assert lookup.shape == (256,) and lookup.dtype == np.uint8
    assert lookup[0] == lookup[16] == 0
    assert lookup[235] == lookup[255] == 255
    valid = np.arange(32, 225)
    recovered = 16.0 + lookup[valid].astype(float) * (219.0 / 255.0)
    np.testing.assert_allclose(recovered, valid, atol=0.5, rtol=0)
    levels = np.rint(np.linspace(32, 224, 16)).astype(np.uint8)
    recovered = 16.0 + lookup[levels].astype(float) * (219.0 / 255.0)
    np.testing.assert_array_equal(np.rint((recovered - 32) / 192 * 15), np.arange(16))


def test_auxiliary_preparation_leaves_scene_rgb_and_alpha_unchanged():
    torch = pytest.importorskip("torch")
    assert not torch.cuda.is_initialized()
    encoder = GpuH264Encoder.__new__(GpuH264Encoder)
    encoder.height_per_eye = 4
    encoder._surface = torch.arange(10 * 16 * 4, dtype=torch.int64).to(torch.uint8).reshape(10, 16, 4)
    encoder._auxiliary_rgb_levels = torch.tensor(_limited_code_to_rgb())
    original = encoder._surface.clone()
    encoder._prepare_auxiliary_rgb()
    assert torch.equal(encoder._surface[:4], original[:4])
    assert torch.equal(encoder._surface[..., 3], original[..., 3])
    expected = encoder._auxiliary_rgb_levels[original[4:, :, 0].long()]
    assert torch.equal(encoder._surface[4:, :, :3], expected[..., None].expand(-1, -1, 3))
    assert not torch.cuda.is_initialized()


def test_encoder_configuration_keeps_latency_profile_and_limited_atlas_contract(monkeypatch):
    torch = pytest.importorskip("torch")
    assert not torch.cuda.is_initialized()
    captured = {}
    codec = SimpleNamespace()

    def create(*args, **options):
        captured.update(args=args, options=options)
        return SimpleNamespace(EndEncode=lambda: None)

    codec.CreateEncoder = create
    fake_torch = SimpleNamespace(
        uint8=torch.uint8, device=torch.device,
        tensor=lambda values, **kw: torch.tensor(values, dtype=kw["dtype"]),
        empty=lambda shape, **kw: torch.empty(shape, dtype=kw["dtype"]),
        cuda=SimpleNamespace(is_available=lambda: True, Stream=lambda **kw: object(),
                             Event=lambda: object()),
    )
    monkeypatch.setitem(sys.modules, "torch", fake_torch)
    monkeypatch.setattr("horus.remote_rendering.nvenc_encoder._load_low_latency_codec", lambda: codec)
    with GpuH264Encoder(256, 256, 64, fps=60, bitrate_kbps=12000):
        pass
    assert captured["args"] == (512, 336, "ABGR", False)
    assert captured["options"] == dict(
        codec="h264", horus_h264_profile="baseline", horus_h264_color_range="limited",
        preset="P1", tuning_info="ultra_low_latency", rc="cbr", fps=60, gop=60,
        idrperiod=60, bf=0, extra_output_delay=0, lookahead=0,
        bitrate=12000000, maxbitrate=12000000, vbvbufsize=200000,
        gpu_id=0, multipass="disabled", colorspace="bt601", repeatspspps=1,
    )
    assert not torch.cuda.is_initialized()


def test_source_patch_preserves_delay_profile_and_opts_into_explicit_vui(tmp_path):
    helper_path = Path(__file__).resolve().parents[1] / "examples/tools/install_remote_nvenc.py"
    spec = importlib.util.spec_from_file_location("nvenc_colour_patch_test", helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    source = tmp_path / "codec"
    package = source / "src/PyNvVideoCodec"
    (package / "src").mkdir(parents=True)
    encoder = package / "src/PyNvEncoder.cpp"
    encoder.write_text("std::make_unique<NvEncoderCuda>(cudacontext, cudastream, _width, _height, eBufferFormat);")
    config = package / "src/NvEncoderClInterface.cpp"
    config.write_text('''        // Need to set up HIGH_444 profile for YUV444 input;
    SetupVuiConfig(config.h264VUIParameters, parent_params,
        print_settings);''')
    (package / "__init__.py").write_text("")
    helper.patch_source(source)
    assert 'options.count("extra_output_delay")' in encoder.read_text()
    patched = config.read_text()
    assert "NV_ENC_H264_PROFILE_BASELINE_GUID" in patched
    assert "NV_ENC_H264_ENTROPY_CODING_MODE_CAVLC" in patched
    assert 'FindAttribute(options, "horus_h264_color_range")' in patched
    assert 'horus_color_range != "limited" && horus_color_range != "full"' in patched
    assert 'parent_params.colorSpace != "bt601" && parent_params.colorSpace != "bt709"' in patched
    assert 'vui.videoSignalTypePresentFlag = 1;' in patched
    assert 'vui.colourDescriptionPresentFlag = 1;' in patched
    assert 'vui.videoFullRangeFlag = (horus_color_range == "full") ? 1 : 0;' in patched
    capabilities = (package / "__init__.py").read_text()
    for name in ("OUTPUT_DELAY", "H264_PROFILE", "H264_COLOR_VUI"):
        assert f"HORUS_{name}_OPTION = 1" in capabilities
