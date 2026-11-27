"""Dense GLIM map: all MID-360 scans placed with GLIM's own trajectory (Philip Stix).

    python tools/glim_dense_map.py --sequence Dynamic01 [--run 0]
    -> results/glim/<seq>/run_<k>/map.dense.ply   (git-ignored by results/**/map.*)

GLIM's map.ply is the merged submaps of its dump, which GLIM keeps strongly downsampled (about 1 % of the points
of the reference map). That lowers completeness and makes wall-noise metrics (MPV/MME) impossible, although the
estimate itself is fine. This tool builds the map the same way as the pseudo-ground truth
(pipeline/reference_map.py on main), but with GLIM's poses instead of the motion capture:

    p_world = T_world_lidar(t) * p_lidar,   T_world_lidar from the dump's traj_lidar.txt (with global optimisation)

Each point gets the pose interpolated at its own time (timebase + offset_time), which removes the motion
distortion within a scan. Same filters as the reference map: Livox noise tag, range 0.5-12 m, 2 cm voxels.
No people filter: that belongs to the reference, not to an estimate. The estimate uses no ground truth.
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pipeline.common import load_sequences  # noqa: E402

POINT = np.dtype([("t", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("refl", "u1"), ("tag", "u1"), ("line", "u1")])


def read_scans(bag: Path, topic: str):
    """Yield (point times [s], xyz in the LiDAR frame) per Livox CustomMsg scan of the original ROS 1 bag."""
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        for _, _, raw in reader.messages(connections=conns):
            raw = bytes(raw)
            n_frame = struct.unpack_from("<I", raw, 12)[0]           # seq, sec, nsec, then frame_id string
            o = 16 + n_frame
            timebase = struct.unpack_from("<Q", raw, o)[0]
            o += 8 + 4 + 1 + 3                                        # timebase, point_num, lidar_id, rsvd[3]
            n = struct.unpack_from("<I", raw, o)[0]
            pts = np.frombuffer(raw, dtype=POINT, count=n, offset=o + 4)
            pts = pts[(pts["tag"] & 0x0F) == 0]                       # Livox tag: drop noise-flagged returns
            yield ((timebase + pts["t"].astype(np.uint64)) * 1e-9,
                   np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64))


def dense_map(seq: str, run: int, rmin: float = 0.5, rmax: float = 12.0, voxel: float = 0.02) -> np.ndarray:
    cfg = load_sequences()
    root = ROOT / cfg["data_root"]
    traj = np.loadtxt(root / seq / "glim_dumps" / f"run_{run:02d}" / "traj_lidar.txt")
    t_traj, keep = np.unique(traj[:, 0], return_index=True)
    pos, slerp = traj[keep, 1:4], Slerp(t_traj, Rotation.from_quat(traj[keep, 4:8]))
    chunks = []
    for t, xyz in read_scans(root / f"{seq}.bag", cfg["topics"]["lidar3d"]):
        rng = np.linalg.norm(xyz, axis=1)
        ok = (rng > rmin) & (rng < rmax) & (t >= t_traj[0]) & (t <= t_traj[-1])
        if not ok.any():
            continue
        t, xyz = t[ok], xyz[ok]
        p = np.column_stack([np.interp(t, t_traj, pos[:, i]) for i in range(3)])
        pw = slerp(t).apply(xyz) + p
        _, first = np.unique(np.floor(pw / voxel).astype(np.int64), axis=0, return_index=True)
        chunks.append(pw[first])
    pts = np.concatenate(chunks)
    _, first = np.unique(np.floor(pts / voxel).astype(np.int64), axis=0, return_index=True)
    return pts[first]


def write_ply(path: Path, xyz: np.ndarray) -> None:
    with open(path, "wb") as f:
        f.write(f"ply\nformat binary_little_endian 1.0\nelement vertex {len(xyz)}\n"
                "property float x\nproperty float y\nproperty float z\nend_header\n".encode())
        f.write(xyz.astype("<f4").tobytes())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sequence", required=True)
    ap.add_argument("--run", type=int, default=0)
    a = ap.parse_args()
    pts = dense_map(a.sequence, a.run)
    out = ROOT / "results" / "glim" / a.sequence / f"run_{a.run:02d}" / "map.dense.ply"
    write_ply(out, pts)
    print(f"{a.sequence}: {len(pts)} points (2 cm voxels) -> {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
