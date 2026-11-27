"""Prepare the shared M3DGR sequences once, so every system reads identical inputs.

    python -m pipeline.prepare_data --bag data/M3DGR/Dynamic01.bag --sequence Dynamic01

Steps:
  1. extract_rgb       compressed RGB images -> data/M3DGR/<seq>/rgb/<t>.jpg + rgb.txt
  2. extract_depth     compressedDepth       -> data/M3DGR/<seq>/depth/<t>.png + depth.txt   (TODO)
  3. convert_ros2      --ros2: ROS 2 (MCAP) copy + PointCloud2 topics next to the Livox CustomMsg topics
  4. ground truth      GT<seq>.txt next to the bag -> data/M3DGR/<seq>/gt_tum.txt (already TUM)

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


LIVOX_POINT = np.dtype([("t", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                        ("refl", "u1"), ("tag", "u1"), ("line", "u1")])
PC2_POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("tag", "u1"),
                      ("line", "u1"), ("pad", "u1", 2), ("t", "<f4"), ("timestamp", "<f8")])


def _livox_scans(bag: Path, topic: str):
    """(bag time ns, header stamp s, frame_id, points) per Livox CustomMsg, parsed from the ROS 1 bytes."""
    import struct
    with AnyReader([bag]) as reader:
        for conn, t_bag, raw in reader.messages(connections=[c for c in reader.connections if c.topic == topic]):
            raw = bytes(raw)
            sec, nsec, n_frame = struct.unpack_from("<III", raw, 4)
            frame = raw[16:16 + n_frame].decode()
            o = 16 + n_frame
            timebase = struct.unpack_from("<Q", raw, o)[0]
            n = struct.unpack_from("<I", raw, o + 16)[0]
            pts = np.frombuffer(raw, dtype=LIVOX_POINT, count=n, offset=o + 20)
            yield t_bag, timebase * 1e-9, frame, pts


def _pointcloud2(typestore, stamp: float, frame: str, pts: np.ndarray):
    T = typestore.types
    data = np.zeros(len(pts), dtype=PC2_POINT)
    for k in ("x", "y", "z", "tag", "line"):
        data[k] = pts[k]
    data["intensity"] = pts["refl"]
    data["t"] = pts["t"] * 1e-9                      # seconds since scan start
    data["timestamp"] = stamp + pts["t"] * 1e-9      # absolute seconds
    PF = T["sensor_msgs/msg/PointField"]
    fields = [PF(name=name, offset=PC2_POINT.fields[name][1], datatype=dt, count=1) for name, dt in
              (("x", 7), ("y", 7), ("z", 7), ("intensity", 7), ("tag", 2), ("line", 2), ("t", 7), ("timestamp", 8))]
    sec = int(stamp)
    header = T["std_msgs/msg/Header"](stamp=T["builtin_interfaces/msg/Time"](sec=sec, nanosec=int((stamp - sec) * 1e9)),
                                      frame_id=frame)
    return T["sensor_msgs/msg/PointCloud2"](header=header, height=1, width=len(data), fields=fields,
                                            is_bigendian=False, point_step=PC2_POINT.itemsize,
                                            row_step=PC2_POINT.itemsize * len(data), data=data.view(np.uint8),
                                            is_dense=True)


def convert_ros2(bag: Path, out_dir: Path, lidar_topics: list[str]) -> Path:
    """ROS 2 (Humble, MCAP) copy of a ROS 1 bag, plus a sensor_msgs/PointCloud2 topic `<topic>_points` next to every
    Livox CustomMsg topic (fields x y z intensity tag line, t = seconds since scan start, timestamp = absolute)."""
    import shutil
    import subprocess
    import sys
    from rosbags.highlevel import AnyReader as Reader2
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_typestore

    tmp = out_dir.with_name(out_dir.name + "_tmp")
    shutil.rmtree(tmp, ignore_errors=True)
    shutil.rmtree(out_dir, ignore_errors=True)
    convert = Path(sys.executable).with_name("rosbags-convert")      # upstream recipe (M3DGR README, section 5.8)
    subprocess.run([str(convert), "--src", str(bag), "--dst", str(tmp), "--src-typestore", "ros1_noetic",
                    "--dst-typestore", "ros2_humble", "--dst-storage", "mcap", "--dst-version", "8"], check=True)
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with Reader2([tmp]) as reader, Writer(out_dir, version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        conns = {}
        for c in reader.connections:
            conns[c.id] = writer.add_connection(c.topic, c.msgtype, msgdef=c.msgdef.data, rihs01=c.digest)
        for c, t, raw in reader.messages():
            writer.write(conns[c.id], t, raw)
        for topic in lidar_topics:
            pc = writer.add_connection(topic + "_points", "sensor_msgs/msg/PointCloud2", typestore=ts)
            for t_bag, stamp, frame, pts in _livox_scans(bag, topic):
                writer.write(pc, t_bag, ts.serialize_cdr(_pointcloud2(ts, stamp, frame, pts), "sensor_msgs/msg/PointCloud2"))
    shutil.rmtree(tmp, ignore_errors=True)
    return out_dir


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--sequence", required=True)
    ap.add_argument("--ros2", action="store_true", help="also write a ROS 2 (MCAP) copy with PointCloud2 LiDAR topics")
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
    if args.ros2:
        out = convert_ros2(args.bag, args.bag.with_name(f"{args.sequence}_ros2"),
                           [cfg["topics"]["lidar3d"], "/livox/avia/lidar"])
        print(f"{args.sequence}: ROS 2 copy -> {out}")


if __name__ == "__main__":
    main()
