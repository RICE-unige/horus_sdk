"""Compare decoded samples against their exact recorded RGB frame identities."""

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np

from real_camera_data import read_index


def quality(captures, source_events, dataset):
    identities = {(row["stream"], row["publish_wall_ns"]): int(row["index"])
                  for path in source_events for row in csv.DictReader(path.open())
                  if row["kind"] in {"rgb", "jpeg"}}
    frames = read_index(dataset / "rgb.txt")
    rows, missing = [], []
    for path in sorted(captures.rglob("quality/cam*-*.png")):
        camera, stamp = path.stem.split("-", 1)
        index = identities.get((camera.removeprefix("cam"), stamp))
        if index is None:
            missing.append(str(path.relative_to(captures)))
            continue
        original = cv2.imread(str(dataset / frames[index][1]))
        received = cv2.imread(str(path))
        if original is None or received is None or original.shape != received.shape:
            raise ValueError(f"Invalid source-matched quality sample: {path}")
        mse = float(np.mean((original.astype(np.float32)-received.astype(np.float32))**2))
        rows.append(dict(run=path.parent.parent.name, camera=camera, stamp=stamp, recorded_index=index,
            psnr_rgb_db=10*np.log10(255**2/mse) if mse else None, identical=mse == 0))
    return dict(metric="Full-resolution RGB PSNR; no alignment, resizing or colour correction",
                samples=rows, unmatched_samples=missing)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", required=True, type=Path)
    parser.add_argument("--source-events", required=True, nargs="+", type=Path)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = quality(args.captures, args.source_events, args.dataset)
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(f"Matched {len(result['samples'])} quality samples; unmatched {len(result['unmatched_samples'])}")


if __name__ == "__main__":
    main()
