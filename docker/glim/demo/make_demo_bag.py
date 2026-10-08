"""Build the ROS 2 bag for the RViz live demo of GLIM (Philip Stix). Not used for results.

    python docker/glim/demo/make_demo_bag.py --bag data/M3DGR/Varying-illu01.bag --sequence Varying-illu01

Writes <data_root>/<sequence>/glim_demo/ with
  * /livox/mid360/lidar, /livox/mid360/imu   same conversion as tools/glim_prepare_bag.py (GLIM input)
  * /camera/color/image_raw/compressed       RGB camera, unchanged (RViz shows it via image_transport)
  * /gt/odom                                  mocap ground truth as nav_msgs/Odometry at ~10 Hz, so RViz
                                              can draw its path next to GLIM's (frame "world")
and <sequence>/glim_demo_tf.txt: the static transform map -> world (SE(3) Umeyama fit of the GT to
GLIM's traj_imu.txt of a previous pipeline run), so both paths are drawn on top of each other.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from glim_prepare_bag import _header, imu_to_ros2, livox_to_pointcloud2  # noqa: E402
from pipeline.common import load_sequences  # noqa: E402

GT_TOPIC = "/vrpn_client_node/UGV/pose"
GT_OUT = "/gt/odom"
GT_PERIOD_NS = 100_000_000   # 10 Hz is plenty for drawing


def write_demo_bag(bag: Path, out_bag: Path, topics: dict) -> dict:
    ts = get_typestore(Stores.ROS2_JAZZY)
    T = ts.types
    lidar, imu, rgb = topics["lidar3d"], topics["imu"], topics["rgb"]
    counts = {lidar: 0, imu: 0, rgb: 0, GT_OUT: 0}
    last_gt = -GT_PERIOD_NS
    with AnyReader([bag]) as reader, Writer(out_bag, version=9) as writer:
        conns = [c for c in reader.connections if c.topic in (lidar, imu, rgb, GT_TOPIC)]
        out = {lidar: writer.add_connection(lidar, "sensor_msgs/msg/PointCloud2", typestore=ts),
               imu: writer.add_connection(imu, "sensor_msgs/msg/Imu", typestore=ts),
               rgb: writer.add_connection(rgb, "sensor_msgs/msg/CompressedImage", typestore=ts),
               GT_TOPIC: writer.add_connection(GT_OUT, "nav_msgs/msg/Odometry", typestore=ts)}
        for conn, t_ns, raw in reader.messages(connections=conns):
            if conn.topic == GT_TOPIC:
                if t_ns - last_gt < GT_PERIOD_NS:
                    continue
                last_gt = t_ns
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == lidar:
                new, msgtype = livox_to_pointcloud2(T, msg), "sensor_msgs/msg/PointCloud2"
            elif conn.topic == imu:
                new, msgtype = imu_to_ros2(T, msg), "sensor_msgs/msg/Imu"
            elif conn.topic == rgb:
                new = T["sensor_msgs/msg/CompressedImage"](header=_header(T, msg.header), format=msg.format,
                                                           data=msg.data)
                msgtype = "sensor_msgs/msg/CompressedImage"
            else:
                new, msgtype = gt_to_odometry(T, msg), "nav_msgs/msg/Odometry"
            writer.write(out[conn.topic], t_ns, ts.serialize_cdr(new, msgtype))
            counts[GT_OUT if conn.topic == GT_TOPIC else conn.topic] += 1
    return counts


def gt_to_odometry(T, msg):
    header = _header(T, msg.header)
    header.frame_id = "world"
    p, q = msg.pose.position, msg.pose.orientation
    pose = T["geometry_msgs/msg/Pose"](position=T["geometry_msgs/msg/Point"](x=p.x, y=p.y, z=p.z),
                                       orientation=T["geometry_msgs/msg/Quaternion"](x=q.x, y=q.y, z=q.z, w=q.w))
    zero3 = T["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)
    return T["nav_msgs/msg/Odometry"](
        header=header, child_frame_id="gt_body",
        pose=T["geometry_msgs/msg/PoseWithCovariance"](pose=pose, covariance=np.zeros(36)),
        twist=T["geometry_msgs/msg/TwistWithCovariance"](
            twist=T["geometry_msgs/msg/Twist"](linear=zero3, angular=zero3), covariance=np.zeros(36)))


def fit_map_to_world(gt_file: Path, est_file: Path, max_dt: float = 0.01) -> np.ndarray:
    """4x4 T_map_world: least-squares SE(3) fit (Umeyama, no scale) of GT positions to GLIM positions."""
    gt, est = np.loadtxt(gt_file), np.loadtxt(est_file)
    idx = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
    idx = np.where(np.abs(gt[idx - 1, 0] - est[:, 0]) < np.abs(gt[idx, 0] - est[:, 0]), idx - 1, idx)
    ok = np.abs(gt[idx, 0] - est[:, 0]) < max_dt
    src, dst = gt[idx[ok], 1:4], est[ok, 1:4]
    mu_s, mu_d = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s))
    S = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ S @ Vt
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R, mu_d - R @ mu_s
    return M


def _quat_xyzw(R: np.ndarray) -> list[float]:
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [x, y, z, w]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", type=Path, required=True, help="original M3DGR ROS 1 bag")
    ap.add_argument("--sequence", required=True)
    ap.add_argument("--run", type=int, default=0, help="pipeline run whose GLIM trajectory is used for the fit")
    a = ap.parse_args()

    seq_cfg = load_sequences()
    seq_dir = ROOT / seq_cfg["data_root"] / a.sequence
    out = seq_dir / "glim_demo"
    if out.exists():
        print(f"[keep] {out} exists")
    else:
        for topic, n in write_demo_bag(a.bag, out, seq_cfg["topics"]).items():
            print(f"{topic}: {n} messages")

    est = seq_dir / "glim_dumps" / f"run_{a.run:02d}" / "traj_imu.txt"
    if not est.exists():
        raise SystemExit(f"{est} missing - run the pipeline once first (python -m pipeline.run_all --system glim ...)")
    M = fit_map_to_world(seq_dir / "gt_tum.txt", est)
    args = [*M[:3, 3], *_quat_xyzw(M[:3, :3])]
    (seq_dir / "glim_demo_tf.txt").write_text(" ".join(f"{v:.6f}" for v in args) + "\n")
    print(f"map -> world: {' '.join(f'{v:.3f}' for v in args)}")


if __name__ == "__main__":
    main()
