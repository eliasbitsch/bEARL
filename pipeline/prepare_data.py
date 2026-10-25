"""Prepare the shared M3DGR sequences once, so every system reads identical inputs.

    python -m pipeline.prepare_data --bag data/M3DGR/Dynamic01.bag --sequence Dynamic01

Steps:
  1. extract_rgb       compressed RGB images -> data/M3DGR/<seq>/rgb/<t>.jpg + rgb.txt
  2. extract_depth     compressedDepth       -> data/M3DGR/<seq>/depth/<t>.png + depth.txt   (TODO)
  3. convert_livox     Livox CustomMsg       -> sensor_msgs/PointCloud2 bag                 (TODO)
  4. ground truth      GT<seq>.txt next to the bag -> data/M3DGR/<seq>/gt_tum.txt (already TUM)
  5. ROS 2 copy        `rosbags-convert --src <bag> --dst <bag>_ros2` for ROS 2 based systems

Owner: data role (see docs/PLAN.md, Section 7).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader

from pipeline.common import ROOT, load_sequences


def extract_rgb(bag: Path, out_dir: Path, topic: str) -> int:
    """Write every compressed RGB frame, plus a TUM-style rgb.txt index.

    JPEG frames are written byte for byte as they are stored in the bag (no re-encoding,
    ~5x smaller than PNG). Any other format is decoded and written as PNG.
    """
    img_dir = out_dir / "rgb"
    img_dir.mkdir(parents=True, exist_ok=True)
    index = ["# timestamp filename"]
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise ValueError(f"topic {topic} not in {bag}")
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            data = bytes(msg.data)
            if data[:3] == b"\xff\xd8\xff":
                name = f"rgb/{stamp:.6f}.jpg"
                (out_dir / name).write_bytes(data)
            else:
                name = f"rgb/{stamp:.6f}.png"
                cv2.imwrite(str(out_dir / name), cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR))
            index.append(f"{stamp:.6f} {name}")
    (out_dir / "rgb.txt").write_text("\n".join(index) + "\n")
    return len(index) - 1


def prepare_ground_truth(src: Path, dst: Path) -> int:
    """Copy the M3DGR mocap GT (already TUM: t x y z qx qy qz qw) to <seq>/gt_tum.txt, sorted by time.

    M3DGR evaluates directly against this file (README, Section 6: `evo_ape tum GT<seq>.txt est.txt`),
    so no marker -> base transform is applied; calibration.md does not provide one.
    """
    gt = np.loadtxt(src, comments="#")
    gt = gt[np.argsort(gt[:, 0], kind="stable")]
    dst.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(dst, gt, fmt="%.6f", header="timestamp tx ty tz qx qy qz qw")
    return len(gt)


def extract_depth(bag: Path, out_dir: Path, topic: str) -> int:
    # compressedDepth = 12-byte header + PNG (16 bit, mm). Decode, write depth/<t>.png + depth.txt.
    raise NotImplementedError("TODO (data role)")


def convert_livox(bag: Path, out_bag: Path, topic: str) -> None:
    # Register livox_ros_driver/CustomMsg with the rosbags typestore, convert every message to
    # sensor_msgs/PointCloud2 with fields x, y, z, intensity, t (per-point offset time).
    raise NotImplementedError("TODO (data role)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--sequence", required=True)
    args = ap.parse_args()

    cfg = load_sequences()
    out_dir = ROOT / cfg["data_root"] / args.sequence
    gt_src = args.bag.parent / f"GT{args.sequence}.txt"
    if gt_src.exists():
        n = prepare_ground_truth(gt_src, out_dir / "gt_tum.txt")
        print(f"{args.sequence}: {n} GT poses -> {out_dir / 'gt_tum.txt'}")
    else:
        print(f"{args.sequence}: {gt_src.name} not found next to the bag, GT skipped")
    n = extract_rgb(args.bag, out_dir, cfg["topics"]["rgb"])
    print(f"{args.sequence}: {n} RGB frames -> {out_dir / 'rgb'}")


if __name__ == "__main__":
    main()
