"""Convert one M3DGR ROS 1 bag into the ROS 2 bag that GLIM reads (Philip Stix).

    python tools/glim_prepare_bag.py --bag data/M3DGR/Dark03.bag --sequence Dark03

GLIM only subscribes to sensor_msgs/PointCloud2, but M3DGR stores the MID-360 as Livox
CustomMsg. This script writes <data_root>/<sequence>/glim_ros2/ (rosbag2, sqlite3, Jazzy format)
containing only the two topics GLIM needs, with the original topic names and receive times:
  * IMU     sensor_msgs/Imu, unchanged (acceleration stays in g, GLIM's acc_scale handles it)
  * LiDAR   sensor_msgs/PointCloud2 with float32 fields x, y, z, intensity, t
            t = per-point time in seconds relative to header.stamp (needed for deskewing)
No points are filtered or reordered, so GLIM sees exactly what the sensor recorded.
Pure Python (rosbags), no ROS installation needed; runs on Windows too.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pipeline.common import load_sequences  # noqa: E402

OUT_NAME = "glim_ros2"
POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("t", "<f4")])


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _header(T, header):
    return T["std_msgs/msg/Header"](
        stamp=T["builtin_interfaces/msg/Time"](sec=header.stamp.sec, nanosec=header.stamp.nanosec),
        frame_id=header.frame_id)


def livox_to_pointcloud2(T, msg):
    """Livox CustomMsg (livox_ros_driver or livox_ros_driver2) -> sensor_msgs/PointCloud2."""
    pts = msg.points
    cloud = np.empty(len(pts), dtype=POINT_DTYPE)
    if len(pts):
        # offset_time is relative to timebase; GLIM expects times relative to header.stamp
        rel0 = (int(msg.timebase) - _stamp_ns(msg.header.stamp)) * 1e-9
        cloud["x"] = [p.x for p in pts]
        cloud["y"] = [p.y for p in pts]
        cloud["z"] = [p.z for p in pts]
        cloud["intensity"] = [p.reflectivity for p in pts]
        cloud["t"] = rel0 + np.array([p.offset_time for p in pts], dtype=np.float64) * 1e-9
    PointField = T["sensor_msgs/msg/PointField"]
    fields = [PointField(name=n, offset=POINT_DTYPE.fields[n][1], datatype=PointField.FLOAT32, count=1)
              for n in POINT_DTYPE.names]
    return T["sensor_msgs/msg/PointCloud2"](
        header=_header(T, msg.header), height=1, width=len(cloud), fields=fields,
        is_bigendian=False, point_step=POINT_DTYPE.itemsize, row_step=POINT_DTYPE.itemsize * len(cloud),
        data=np.frombuffer(cloud.tobytes(), dtype=np.uint8), is_dense=True)


def imu_to_ros2(T, msg):
    return T["sensor_msgs/msg/Imu"](
        header=_header(T, msg.header),
        orientation=T["geometry_msgs/msg/Quaternion"](x=msg.orientation.x, y=msg.orientation.y,
                                                       z=msg.orientation.z, w=msg.orientation.w),
        orientation_covariance=msg.orientation_covariance,
        angular_velocity=T["geometry_msgs/msg/Vector3"](x=msg.angular_velocity.x, y=msg.angular_velocity.y,
                                                         z=msg.angular_velocity.z),
        angular_velocity_covariance=msg.angular_velocity_covariance,
        linear_acceleration=T["geometry_msgs/msg/Vector3"](x=msg.linear_acceleration.x,
                                                            y=msg.linear_acceleration.y,
                                                            z=msg.linear_acceleration.z),
        linear_acceleration_covariance=msg.linear_acceleration_covariance)


def convert(bag: Path, out_bag: Path, lidar_topic: str, imu_topic: str) -> dict:
    if out_bag.exists():
        raise SystemExit(f"{out_bag} already exists - delete it to convert again")
    ts = get_typestore(Stores.ROS2_JAZZY)
    T = ts.types
    counts = {lidar_topic: 0, imu_topic: 0}
    with AnyReader([bag]) as reader, Writer(out_bag, version=9) as writer:
        conns = [c for c in reader.connections if c.topic in counts]
        found = {c.topic: c.msgtype for c in conns}
        if set(found) != set(counts):
            raise SystemExit(f"missing topics in {bag}: {set(counts) - set(found)}; "
                             f"available: {sorted({c.topic for c in reader.connections})}")
        if not found[lidar_topic].endswith("CustomMsg"):
            raise SystemExit(f"{lidar_topic} is {found[lidar_topic]}, expected a Livox CustomMsg")
        out = {lidar_topic: writer.add_connection(lidar_topic, "sensor_msgs/msg/PointCloud2", typestore=ts),
               imu_topic: writer.add_connection(imu_topic, "sensor_msgs/msg/Imu", typestore=ts)}
        for conn, t_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == lidar_topic:
                new, msgtype = livox_to_pointcloud2(T, msg), "sensor_msgs/msg/PointCloud2"
            else:
                new, msgtype = imu_to_ros2(T, msg), "sensor_msgs/msg/Imu"
            writer.write(out[conn.topic], t_ns, ts.serialize_cdr(new, msgtype))
            counts[conn.topic] += 1
    return counts


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True, help="original M3DGR ROS 1 bag")
    ap.add_argument("--sequence", required=True, help="sequence name from config/sequences.yaml")
    ap.add_argument("--out", type=Path, help=f"default: <data_root>/<sequence>/{OUT_NAME}")
    a = ap.parse_args()

    seq_cfg = load_sequences()
    out = a.out or ROOT / seq_cfg["data_root"] / a.sequence / OUT_NAME
    out.parent.mkdir(parents=True, exist_ok=True)
    counts = convert(a.bag, out, seq_cfg["topics"]["lidar3d"], seq_cfg["topics"]["imu"])
    for topic, n in counts.items():
        print(f"{topic}: {n} messages")
    print(f"-> {out}")


if __name__ == "__main__":
    main()
