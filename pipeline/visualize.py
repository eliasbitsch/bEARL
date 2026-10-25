"""Write a 3D replay of one run for Foxglove / Lichtblick: GT vs. estimate, camera image, error over time.

    python -m pipeline.visualize --sequence Dynamic01 --est results/vggt_long/Dynamic01/run_00/trajectory_tum.txt --alignment sim3
    python -m pipeline.visualize --sequence Dynamic01 --wheel-odom          # sanity check without any SLAM run

Output: an MCAP file (ROS 2 messages) next to the estimate, or data/M3DGR/<seq>/viz_wheel_odom.mcap.
Open it in Lichtblick/Foxglove and import tools/lichtblick_layout.json (Layout menu -> Import from file).

Topics (all in frame "map" = mocap world frame):
  /gt/path, /est/path        full trajectories (est aligned to GT, Umeyama 1991, same as pipeline.evaluate)
  /gt/pose, /est/pose        current pose, plus TF map -> gt_body / est_body
  /est/error_line            line between the current est and GT position = the per-pose ATE term
  /est/position_error        that error in metres (std_msgs/Float64), for the Plot panel
  /camera/image/compressed   RGB image as recorded
  /camera/brightness         mean grey value 0-255 of each image (a measurement, not a metric)
  /est/map                   optional: point cloud of the SLAM map (--map file.ply), same alignment

The visualisation is for plausibility checks only; numbers for the paper come from pipeline.evaluate.
"""
from __future__ import annotations

import argparse
import copy
from pathlib import Path

import cv2
import numpy as np
from evo.core import lie_algebra as lie
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

from pipeline.common import ROOT, load_sequences

TS = get_typestore(Stores.ROS2_HUMBLE)
T = TS.types
POSE_RATE = 30.0     # Hz for /tf and current poses
PATH_RATE = 10.0     # Hz of the poses inside the path messages
PATH_EVERY = 2.0     # s between repeated full-path messages (so seeking always shows them)


def _stamp(t: float):
    sec = int(t)
    return T["builtin_interfaces/msg/Time"](sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def _header(t: float, frame: str = "map"):
    return T["std_msgs/msg/Header"](stamp=_stamp(t), frame_id=frame)


def _pose(p: np.ndarray, q_wxyz: np.ndarray):
    P, Q, Pt = T["geometry_msgs/msg/Pose"], T["geometry_msgs/msg/Quaternion"], T["geometry_msgs/msg/Point"]
    return P(position=Pt(x=float(p[0]), y=float(p[1]), z=float(p[2])),
             orientation=Q(w=float(q_wxyz[0]), x=float(q_wxyz[1]), y=float(q_wxyz[2]), z=float(q_wxyz[3])))


def _downsample(traj: PoseTrajectory3D, rate: float) -> np.ndarray:
    """Indices of poses at most `rate` Hz apart."""
    keep, last = [], -np.inf
    for i, t in enumerate(traj.timestamps):
        if t - last >= 1.0 / rate:
            keep.append(i)
            last = t
    return np.array(keep)


def _path_msg(traj: PoseTrajectory3D, idx: np.ndarray, t: float):
    PS = T["geometry_msgs/msg/PoseStamped"]
    poses = [PS(header=_header(traj.timestamps[i]), pose=_pose(traj.positions_xyz[i], traj.orientations_quat_wxyz[i]))
             for i in idx]
    return T["nav_msgs/msg/Path"](header=_header(t), poses=poses)


def wheel_odom_tum(bag: Path, topic: str, out: Path) -> Path:
    """Extract the wheel odometry of the bag as a TUM trajectory (a real, drifting dead-reckoning estimate).

    The odometry is planar. In M3DGR, pose.position.z of /odom is not a height (it grows from ~30 to ~58
    over Dynamic01), so z is set to 0.
    """
    rows = []
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, 0.0, q.x, q.y, q.z, q.w])
    np.savetxt(out, np.array(rows), fmt="%.6f", header="timestamp tx ty tz qx qy qz qw")
    return out


def align_full(gt: PoseTrajectory3D, est: PoseTrajectory3D, sim3: bool, max_diff: float):
    """Umeyama alignment estimated on the associated poses, applied to the full estimate (as evo does).

    Returns the aligned estimate and (s, R, t) with p_aligned = R @ (s * p) + t.
    """
    ref_a, est_a = sync.associate_trajectories(gt, est, max_diff=max_diff)
    r, t, s = copy.deepcopy(est_a).align(ref_a, correct_scale=sim3)
    s = float(s) if sim3 else 1.0
    est = copy.deepcopy(est)
    est.scale(s)
    est.transform(lie.se3(r, t))
    return est, (s, np.asarray(r), np.asarray(t))


def build(seq: str, est_file: Path, sim3: bool, out: Path, map_file: Path | None, label: str) -> None:
    cfg = load_sequences()
    seq_dir = ROOT / cfg["data_root"] / seq
    bag = ROOT / cfg["data_root"] / f"{seq}.bag"
    max_diff = cfg["evaluation"]["t_max_diff"]

    gt = file_interface.read_tum_trajectory_file(str(seq_dir / "gt_tum.txt"))
    est = file_interface.read_tum_trajectory_file(str(est_file))
    est, sim = align_full(gt, est, sim3, max_diff)
    scale = sim[0]

    # per-pose position error after alignment (the term ATE averages over)
    ref_a, est_a = sync.associate_trajectories(gt, est, max_diff=max_diff)
    err = np.linalg.norm(ref_a.positions_xyz - est_a.positions_xyz, axis=1)
    print(f"{label} on {seq}: {'Sim(3)' if sim3 else 'SE(3)'} alignment, scale {scale:.3f}, "
          f"{len(err)} matched poses, ATE RMSE {np.sqrt(np.mean(err**2)):.3f} m (check: pipeline.evaluate is the reference)")

    msgs: list[tuple[str, str, float, object]] = []   # (topic, type, time, msg)
    t0, t1 = gt.timestamps[0], gt.timestamps[-1]

    for traj, name in ((gt, "gt"), (est, "est")):
        idx_path = _downsample(traj, PATH_RATE)
        full = _path_msg(traj, idx_path, t0)
        for t in np.arange(t0, t1, PATH_EVERY):
            msgs.append((f"/{name}/path", "nav_msgs/msg/Path", t, full))
        for i in _downsample(traj, POSE_RATE):
            t = traj.timestamps[i]
            pose = _pose(traj.positions_xyz[i], traj.orientations_quat_wxyz[i])
            msgs.append((f"/{name}/pose", "geometry_msgs/msg/PoseStamped", t,
                         T["geometry_msgs/msg/PoseStamped"](header=_header(t), pose=pose)))
            tf = T["geometry_msgs/msg/TransformStamped"](
                header=_header(t), child_frame_id=f"{name}_body",
                transform=T["geometry_msgs/msg/Transform"](
                    translation=T["geometry_msgs/msg/Vector3"](x=pose.position.x, y=pose.position.y, z=pose.position.z),
                    rotation=pose.orientation))
            msgs.append(("/tf", "tf2_msgs/msg/TFMessage", t, T["tf2_msgs/msg/TFMessage"](transforms=[tf])))

    Pt, C = T["geometry_msgs/msg/Point"], T["std_msgs/msg/ColorRGBA"]
    for i in _downsample(est_a, POSE_RATE):
        t = est_a.timestamps[i]
        a, b = est_a.positions_xyz[i], ref_a.positions_xyz[i]
        line = T["visualization_msgs/msg/Marker"](
            header=_header(t), ns="error", id=0, type=5, action=0,               # 5 = LINE_LIST
            pose=_pose(np.zeros(3), np.array([1.0, 0, 0, 0])),
            scale=T["geometry_msgs/msg/Vector3"](x=0.02, y=0.0, z=0.0),
            color=C(r=1.0, g=0.2, b=0.2, a=1.0),
            lifetime=T["builtin_interfaces/msg/Duration"](sec=0, nanosec=200_000_000), frame_locked=False,
            points=[Pt(x=float(a[0]), y=float(a[1]), z=float(a[2])), Pt(x=float(b[0]), y=float(b[1]), z=float(b[2]))],
            colors=[], texture_resource="", texture=T["sensor_msgs/msg/CompressedImage"](
                header=_header(t), format="", data=np.zeros(0, np.uint8)),
            uv_coordinates=[], text="", mesh_resource="",
            mesh_file=T["visualization_msgs/msg/MeshFile"](filename="", data=np.zeros(0, np.uint8)),
            mesh_use_embedded_materials=False)
        msgs.append(("/est/error_line", "visualization_msgs/msg/Marker", t, line))
        msgs.append(("/est/position_error", "std_msgs/msg/Float64", t, T["std_msgs/msg/Float64"](data=float(err[i]))))

    # camera images + brightness, straight from the bag
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == cfg["topics"]["rgb"]]
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            data = np.frombuffer(bytes(m.data), np.uint8)
            msgs.append(("/camera/image/compressed", "sensor_msgs/msg/CompressedImage", t,
                         T["sensor_msgs/msg/CompressedImage"](header=_header(t, "camera"), format="jpeg", data=data)))
            grey = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)
            msgs.append(("/camera/brightness", "std_msgs/msg/Float64", t,
                         T["std_msgs/msg/Float64"](data=float(grey.mean()))))

    if map_file is not None:
        cloud = _cloud(map_file, sim, t0)
        for t in np.arange(t0, t1, 10.0):        # repeated so that seeking always shows it
            msgs.append(("/est/map", "sensor_msgs/msg/PointCloud2", t, cloud))

    if out.exists():
        out.unlink()
    with Writer(out.with_suffix(""), version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        conns = {}
        for topic, msgtype, t, msg in sorted(msgs, key=lambda m: m[2]):
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, msgtype, typestore=TS)
            writer.write(conns[topic], int(t * 1e9), TS.serialize_cdr(msg, msgtype))
    print(f"-> {out.with_suffix('')}  (open the .mcap inside in Lichtblick / Foxglove)")


def _read_ply_xyzrgb(ply: Path) -> tuple[np.ndarray, np.ndarray]:
    """Binary little-endian PLY with float x, y, z and uchar red, green, blue (as written by VGGT-Long)."""
    with open(ply, "rb") as f:
        header = b""
        while not header.endswith(b"end_header\n"):
            header += f.read(1)
        n = int(header.split(b"element vertex ")[1].split(b"\n")[0])
        rec = np.fromfile(f, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                    ("r", "u1"), ("g", "u1"), ("b", "u1")], count=n)
    return np.column_stack([rec["x"], rec["y"], rec["z"]]), np.column_stack([rec["r"], rec["g"], rec["b"]])


def _cloud(ply: Path, sim: tuple, t: float, voxel: float = 0.03):
    """Map points in the estimate's frame -> same Sim(3)/SE(3) as the trajectory, thinned to one point per voxel."""
    s, r, trans = sim
    xyz, rgb = _read_ply_xyzrgb(ply)
    xyz = (s * xyz) @ r.T + trans
    _, keep = np.unique(np.floor(xyz / voxel).astype(np.int64), axis=0, return_index=True)
    xyz, rgb = xyz[keep].astype(np.float32), rgb[keep].astype(np.uint32)
    packed = ((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]).view(np.float32)
    data = np.zeros(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
    data["x"], data["y"], data["z"], data["rgb"] = xyz[:, 0], xyz[:, 1], xyz[:, 2], packed
    PF = T["sensor_msgs/msg/PointField"]
    fields = [PF(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate(("x", "y", "z", "rgb"))]  # 7 = FLOAT32
    print(f"map: {len(rgb)} points after {voxel * 100:.0f} cm voxel thinning")
    return T["sensor_msgs/msg/PointCloud2"](header=_header(t), height=1, width=len(data), fields=fields,
                                            is_bigendian=False, point_step=16, row_step=16 * len(data),
                                            data=data.view(np.uint8), is_dense=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sequence", required=True)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--est", type=Path, help="estimated trajectory, TUM format")
    src.add_argument("--wheel-odom", action="store_true", help="use the bag's wheel odometry as the estimate")
    ap.add_argument("--alignment", choices=["se3", "sim3"], default="se3")
    ap.add_argument("--map", type=Path, default=None, help="optional map point cloud (.ply) of the run")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()

    cfg = load_sequences()
    seq_dir = ROOT / cfg["data_root"] / a.sequence
    if a.wheel_odom:
        est = wheel_odom_tum(ROOT / cfg["data_root"] / f"{a.sequence}.bag", cfg["topics"]["wheel_odom"],
                             seq_dir / "wheel_odom_tum.txt")
        out, label = a.out or seq_dir / "viz_wheel_odom.mcap", "wheel odometry"
    else:
        est = a.est
        out, label = a.out or est.with_name("viz.mcap"), str(est)
    build(a.sequence, est, a.alignment == "sim3", out, a.map, label)


if __name__ == "__main__":
    main()
