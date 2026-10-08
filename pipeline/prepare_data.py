"""Prepare the shared M3DGR sequences once, so every system reads identical inputs.

    python -m pipeline.prepare_data --bag data/M3DGR/Dark03.bag --sequence Dark03

Steps:
  1. extract_rgb       compressed RGB images -> data/M3DGR/<seq>/rgb/<t>.png + rgb.txt
  2. extract_depth     compressedDepth       -> data/M3DGR/<seq>/depth/<t>.png + depth.txt   (TODO)
  3. convert_livox     Livox CustomMsg       -> sensor_msgs/PointCloud2 bag                 (TODO)
  4. ground truth      dataset GT            -> data/M3DGR/<seq>/gt_tum.txt (TUM format, base frame)
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
    """Write every compressed RGB frame as PNG, plus a TUM-style rgb.txt index."""
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
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            name = f"rgb/{stamp:.6f}.png"
            cv2.imwrite(str(out_dir / name), img)
            index.append(f"{stamp:.6f} {name}")
    (out_dir / "rgb.txt").write_text("\n".join(index) + "\n")
    return len(index) - 1


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
    n = extract_rgb(args.bag, out_dir, cfg["topics"]["rgb"])
    print(f"{args.sequence}: {n} RGB frames -> {out_dir / 'rgb'}")


if __name__ == "__main__":
    main()
