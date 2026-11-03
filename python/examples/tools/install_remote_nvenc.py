#!/usr/bin/env python3
"""Build an isolated PyNvVideoCodec with low-latency WebRTC options.

NVIDIA's MIT-licensed 2.2.2 sources default to three extra output frames.
This patch exposes that existing constructor argument, H.264 Baseline and VUI;
unrelated encoders retain the upstream defaults. Stock site-packages is untouched.
"""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request
import zipfile


VERSION = "2.2.2"
SHA256 = "0ecb47da0bd40dc70e1e684fa95073fd6f3debc607fd4a5056b99b42af5cbeb1"
URL = ("https://api.ngc.nvidia.com/v2/resources/nvidia/pynvvideocodec/versions/"
       f"{VERSION}/files/PyNvVideoCodec_{VERSION}.zip")
ROOT = Path(os.environ.get("HORUS_NVENC_ROOT", str(
    Path.home() / ".cache/horus/remote_render/nvenc-low-latency-v2"))).expanduser()


def patch_source(source):
    path = source / "src/PyNvVideoCodec/src/PyNvEncoder.cpp"
    old = "std::make_unique<NvEncoderCuda>(cudacontext, cudastream, _width, _height, eBufferFormat);"
    new = ("std::make_unique<NvEncoderCuda>(cudacontext, cudastream, _width, _height, "
           "eBufferFormat, options.count(\"extra_output_delay\") ? "
           "static_cast<uint32_t>(std::clamp(std::stoi(options.at(\"extra_output_delay\")), 0, 16)) : 3);")
    text = path.read_text()
    if text.count(old) != 1:
        raise RuntimeError("Pinned NVIDIA encoder constructor did not match")
    path.write_text(text.replace(old, new))
    config = source / "src/PyNvVideoCodec/src/NvEncoderClInterface.cpp"
    text = config.read_text()
    old = "        // Need to set up HIGH_444 profile for YUV444 input;"
    new = '''        // Match the Android WebRTC receiver's advertised AVC profile.
        auto h264_profile = FindAttribute(options, "horus_h264_profile");
        if (!h264_profile.empty()) {
            if (h264_profile != "baseline" ||
                config.encodeCodecConfig.h264Config.chromaFormatIDC == 3) {
                throw std::invalid_argument("Unsupported HORUS H264 profile/input");
            }
            config.profileGUID = NV_ENC_H264_PROFILE_BASELINE_GUID;
            config.encodeCodecConfig.h264Config.entropyCodingMode =
                NV_ENC_H264_ENTROPY_CODING_MODE_CAVLC;
            config.encodeCodecConfig.h264Config.adaptiveTransformMode =
                NV_ENC_H264_ADAPTIVE_TRANSFORM_DISABLE;
        }

''' + old
    if text.count(old) != 1:
        raise RuntimeError("Pinned NVIDIA H264 profile configuration did not match")
    text = text.replace(old, new)
    old = '''    SetupVuiConfig(config.h264VUIParameters, parent_params,
        print_settings);'''
    new = old + '''

    // Opt in without changing the upstream raw/transcoding defaults.
    auto horus_color_range = FindAttribute(options, "horus_h264_color_range");
    if (!horus_color_range.empty()) {
        if ((horus_color_range != "limited" && horus_color_range != "full") ||
            (parent_params.colorSpace != "bt601" && parent_params.colorSpace != "bt709")) {
            throw std::invalid_argument("Unsupported HORUS H264 color range/space");
        }
        auto& vui = config.h264VUIParameters;
        vui.videoSignalTypePresentFlag = 1;
        vui.videoFormat = NV_ENC_VUI_VIDEO_FORMAT_UNSPECIFIED;
        vui.videoFullRangeFlag = (horus_color_range == "full") ? 1 : 0;
        vui.colourDescriptionPresentFlag = 1;
        vui.colourPrimaries = FromString<NV_ENC_VUI_COLOR_PRIMARIES>(parent_params.colorSpace);
        vui.transferCharacteristics = FromString<NV_ENC_VUI_TRANSFER_CHARACTERISTIC>(parent_params.colorSpace);
        vui.colourMatrix = FromString<NV_ENC_VUI_MATRIX_COEFFS>(parent_params.colorSpace);
    }
'''
    if text.count(old) != 1:
        raise RuntimeError("Pinned NVIDIA H264 VUI configuration did not match")
    config.write_text(text.replace(old, new))
    initializer = source / "src/PyNvVideoCodec/__init__.py"
    with initializer.open("a") as output:
        output.write("\n# HORUS patch: exposed NvEncoderCuda's existing output-delay argument.\n"
                     "HORUS_OUTPUT_DELAY_OPTION = 1\n"
                     "HORUS_H264_PROFILE_OPTION = 1\n"
                     "HORUS_H264_COLOR_VUI_OPTION = 1\n")


def main():
    if not shutil.which("nvcc"):
        raise RuntimeError("The CUDA toolkit (nvcc) is required to build the GPU encoder")
    ROOT.mkdir(parents=True, exist_ok=True)
    archive = ROOT / f"PyNvVideoCodec_{VERSION}.zip"
    if not archive.exists():
        print(f"[nvenc] downloading pinned NVIDIA source: {URL}", flush=True)
        temporary = archive.with_suffix(".part")
        with urllib.request.urlopen(URL, timeout=90) as response, temporary.open("wb") as output:
            shutil.copyfileobj(response, output)
        temporary.replace(archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != SHA256:
        raise RuntimeError(f"NVIDIA source checksum mismatch: {archive}")
    source = ROOT / f"PyNvVideoCodec_{VERSION}"
    with zipfile.ZipFile(archive) as bundle:
        for member in bundle.infolist():
            resolved = (ROOT / member.filename).resolve()
            if not resolved.is_relative_to(source.resolve()):
                raise RuntimeError("Unexpected path in NVIDIA source archive")
        bundle.extractall(ROOT)
    patch_source(source)
    env = dict(os.environ, CMAKE_BUILD_PARALLEL_LEVEL="4", MAX_JOBS="4")
    wheels = ROOT / "wheels"
    subprocess.run([sys.executable, "-m", "pip", "wheel", "--no-deps",
                    "--wheel-dir", str(wheels), str(source)], env=env, check=True)
    wheel = max(wheels.glob("pynvvideocodec-*.whl"), key=lambda p: p.stat().st_mtime)
    target = ROOT / "site-packages"
    subprocess.run([sys.executable, "-m", "pip", "install", "--no-deps",
                    "--upgrade", "--target", str(target), str(wheel)], check=True)
    (ROOT / "provenance.json").write_text(json.dumps({
        "upstream": URL, "sha256": SHA256, "license": "MIT",
        "patch": "extra_output_delay, optional H264 baseline and explicit color VUI; upstream defaults unchanged",
        "patch_helper_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "wheel": wheel.name,
    }, indent=2) + "\n")
    print(f"[nvenc] isolated encoder installed: {target}")


if __name__ == "__main__":
    main()
