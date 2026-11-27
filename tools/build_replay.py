"""One 3D replay per sequence with all systems side by side, for Lichtblick / Foxglove (notebook section 6).

    python tools/build_replay.py                      # all three sequences, ~1-2 min each
    python tools/build_replay.py --sequence Varying-illu01

Needs data/M3DGR (bags, GT, reference maps) and the runs in results/. Writes results/replay/<seq>.mcap (git-ignored,
~50 MB each). Open it with `python tools/viewer.py` or inside notebooks/story.py; layout: tools/replay_layout.json.

Topics (frame "map" = mocap world frame; estimates aligned like pipeline.evaluate):
  /gt/{path,pose,map}                 ground truth, pseudo-GT LiDAR map
  /<slug>/{path,pose,map}             per system, plus TF map -> <slug>_body
  /<slug>/position_error              per-pose ATE term [m]
  /vggt_long/confidence               mean VGGT depth confidence per frame (1.0 = floor)
  /camera/image/compressed            RGB at 10 Hz, 320x240;  /camera/brightness  mean grey value 0-255
  /robot/*                            driving state (pipeline.visualize.robot_state)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from evo.core import sync
from evo.tools import file_interface
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pipeline.common import load_sequences  # noqa: E402
from pipeline.visualize import (POSE_RATE, PATH_RATE, PATH_EVERY, MAP_EVERY, TS, T, _cloud, _downsample,  # noqa: E402
                                _header, _path_msg, _pose, align_full, robot_state)

SYSTEMS = {"vggt_long": True, "mast3r_slam": True, "fast_lio2": False}   # slug -> Sim(3) alignment
CAM_RATE, CAM_SIZE = 10.0, (320, 240)
MAP_VOXEL = 0.05
OUT = ROOT / "results" / "replay"
RUNS = pd.read_csv(ROOT / "results" / "cache" / "runs.csv")


def run_dir(slug: str, seq: str) -> Path:
    # VGGT-Long is deterministic and only run_00 has map.ply and frame_confidence.csv
    run = "run_00" if slug == "vggt_long" else RUNS.query("slug == @slug and sequence == @seq").run.iloc[0]
    return ROOT / "results" / slug / seq / run


def poses(name: str, traj, t0: float, t1: float) -> list:
    out = []
    full = _path_msg(traj, _downsample(traj, PATH_RATE), t0)
    out += [(f"/{name}/path", "nav_msgs/msg/Path", t, full) for t in np.arange(t0, t1, PATH_EVERY)]
    for i in _downsample(traj, POSE_RATE):
        t = traj.timestamps[i]
        pose = _pose(traj.positions_xyz[i], traj.orientations_quat_wxyz[i])
        out.append((f"/{name}/pose", "geometry_msgs/msg/PoseStamped", t,
                    T["geometry_msgs/msg/PoseStamped"](header=_header(t), pose=pose)))
        tf = T["geometry_msgs/msg/TransformStamped"](
            header=_header(t), child_frame_id=f"{name}_body",
            transform=T["geometry_msgs/msg/Transform"](
                translation=T["geometry_msgs/msg/Vector3"](x=pose.position.x, y=pose.position.y, z=pose.position.z),
                rotation=pose.orientation))
        out.append(("/tf", "tf2_msgs/msg/TFMessage", t, T["tf2_msgs/msg/TFMessage"](transforms=[tf])))
    return out


def scalar(topic: str, t: np.ndarray, v: np.ndarray) -> list:
    F = T["std_msgs/msg/Float64"]
    return [(topic, "std_msgs/msg/Float64", float(a), F(data=float(b))) for a, b in zip(t, v)]


def build(seq: str) -> Path:
    cfg = load_sequences()
    data = ROOT / cfg["data_root"]
    max_diff = cfg["evaluation"]["t_max_diff"]
    gt = file_interface.read_tum_trajectory_file(str(data / seq / "gt_tum.txt"))
    t0, t1 = gt.timestamps[0], gt.timestamps[-1]
    msgs = poses("gt", gt, t0, t1)

    for slug, sim3 in SYSTEMS.items():
        d = run_dir(slug, seq)
        est, sim = align_full(gt, file_interface.read_tum_trajectory_file(str(d / "trajectory_tum.txt")), sim3, max_diff)
        r, e = sync.associate_trajectories(gt, est, max_diff=max_diff)
        err = np.linalg.norm(r.positions_xyz - e.positions_xyz, axis=1)
        print(f"{seq} {slug} {d.name}: ATE {np.sqrt(np.mean(err ** 2)):.3f} m")
        msgs += poses(slug, est, t0, t1) + scalar(f"/{slug}/position_error", e.timestamps, err)
        maps = [d] + sorted(d.parent.glob("run_*"))
        m = next((c / "map.ply" for c in maps if (c / "map.ply").exists()), None)
        if m is not None:
            if m.parent != d:   # map of another run needs that run's own alignment
                _, sim = align_full(gt, file_interface.read_tum_trajectory_file(str(m.parent / "trajectory_tum.txt")),
                                    sim3, max_diff)
            cloud = _cloud(m, sim, t0, MAP_VOXEL)
            msgs += [(f"/{slug}/map", "sensor_msgs/msg/PointCloud2", t, cloud) for t in np.arange(t0, t1, MAP_EVERY)]

    conf = run_dir("vggt_long", seq) / "frame_confidence.csv"
    if conf.exists():
        # mean over the overlapping chunks that contain a frame, as in the paper figure
        c = pd.read_csv(conf).groupby("image", as_index=False).depth_conf_mean.mean()
        msgs += scalar("/vggt_long/confidence", c.image.str[:-4].astype(float).to_numpy(), c.depth_conf_mean.to_numpy())

    ref = data / seq / "reference_map.ply"
    if ref.exists():
        cloud = _cloud(ref, (1.0, np.eye(3), np.zeros(3)), t0, MAP_VOXEL)
        msgs += [("/gt/map", "sensor_msgs/msg/PointCloud2", t, cloud) for t in np.arange(t0, t1, MAP_EVERY)]

    bag = data / f"{seq}.bag"
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == cfg["topics"]["rgb"]]
        last = -np.inf
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
            msgs += scalar("/camera/brightness", [t], [cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).mean()])
            if t - last < 1.0 / CAM_RATE:
                continue
            last = t
            jpg = cv2.imencode(".jpg", cv2.resize(img, CAM_SIZE, interpolation=cv2.INTER_AREA),
                               [cv2.IMWRITE_JPEG_QUALITY, 75])[1].ravel()
            msgs.append(("/camera/image/compressed", "sensor_msgs/msg/CompressedImage", t,
                         T["sensor_msgs/msg/CompressedImage"](header=_header(t, "camera"), format="jpeg", data=jpg)))
    msgs += robot_state(bag, gt, cfg)

    OUT.mkdir(parents=True, exist_ok=True)
    tmp = OUT / f"{seq}_bag"
    if tmp.exists():
        for f in tmp.iterdir():
            f.unlink()
        tmp.rmdir()
    with Writer(tmp, version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        cs = {}
        for topic, msgtype, t, msg in sorted(msgs, key=lambda m: m[2]):
            if topic not in cs:
                cs[topic] = writer.add_connection(topic, msgtype, typestore=TS)
            writer.write(cs[topic], int(t * 1e9), TS.serialize_cdr(msg, msgtype))
    out = OUT / f"{seq}.mcap"
    out.unlink(missing_ok=True)
    next(tmp.glob("*.mcap")).rename(out)
    for f in tmp.iterdir():
        f.unlink()
    tmp.rmdir()
    print(f"-> {out}  ({out.stat().st_size / 1e6:.0f} MB)")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sequence", choices=[s["name"] for s in load_sequences()["sequences"]])
    a = ap.parse_args()
    for seq in [a.sequence] if a.sequence else [s["name"] for s in load_sequences()["sequences"]]:
        build(seq)


if __name__ == "__main__":
    main()
