"""Pseudo-ground-truth map: accumulate the Livox MID-360 scans along the motion-capture trajectory.

    python -m pipeline.reference_map --sequence Dynamic01                     # -> data/M3DGR/<seq>/reference_map.ply
    python -m pipeline.reference_map --sequence Dynamic01 --every 10 --out x.ply   # quick look with every 10th scan

M3DGR provides no reference map, so we build one from the robot's own LiDAR, placed with the mocap poses
(suggested by the lecturer). Every point gets the GT pose interpolated at its own timestamp
(timebase + offset_time), which also removes the motion distortion within a 0.1 s scan.

    p_world = T_world_body(t) * T_body_base * T_base_lidar * p_lidar

* T_base_lidar from calibration.md: wheel(base) -> camera_imu (body_T_wheel) and mid360 -> camera_imu.
* T_body_base is NOT in the dataset. The mocap body x axis points backwards (driving direction is at ~ -170 deg
  to body x on Dynamic01 and Sha-turn01), so the start value is a 180 deg yaw; the residual is refined separately.

Limitations to state in the paper: "pseudo ground truth" - LiDAR-based (favours LiDAR systems), software-synchronised
sensors, estimated body->base transform, LiDAR range noise of a few cm.
"""
from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation, Slerp

from pipeline.common import ROOT, load_sequences

# calibration.md, section 1.5: mid360 -> camera_imu (p_cimu = R p_mid360 + t)
R_CIMU_MID360 = np.array([[0.0981574, -0.995106, 0.0113748],
                          [0.514299, 0.0409386, -0.856633],
                          [0.851975, 0.0899349, 0.515801]])
T_CIMU_MID360 = np.array([-0.00349766, -0.417688, 0.198242])
# calibration.md, section 1.8: body_T_wheel = wheel(base) -> camera_imu
CIMU_T_WHEEL = np.array([[4.2873564019253907e-02, -9.9906999607154057e-01, 4.5826256555663858e-03, 1.0000278019634017e-00],
                         [2.3548883729155812e-02, -3.5750257528033291e-03, -9.9971629438855181e-01, 0.0477569625897234e-01],
                         [9.9880293731215963e-01, 4.2969316267296165e-02, 2.3373709079293481e-02, 2.0902387796334685e-01],
                         [0.0, 0.0, 0.0, 1.0]])

POINT = np.dtype([("t", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("refl", "u1"), ("tag", "u1"), ("line", "u1")])


def _se3(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    return m


def base_T_lidar() -> np.ndarray:
    """Rotation from the calibration chain. The translation of body_T_wheel (x = 1.0 m) is implausible for this
    robot, so only the rotation of the chain is used; the lever arm is part of T_body_base."""
    r_base_cimu = CIMU_T_WHEEL[:3, :3].T
    return _se3(r_base_cimu @ R_CIMU_MID360, r_base_cimu @ T_CIMU_MID360)


def body_T_base(yaw_deg: float = 180.0, xyz=(0.0, 0.0, 0.0), roll_deg: float = 0.0, pitch_deg: float = 0.0) -> np.ndarray:
    return _se3(Rotation.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix(), np.asarray(xyz))


def read_scans(bag: Path, topic: str, every: int = 1):
    """Yield (point times [s], xyz [N,3] in the LiDAR frame) per scan, parsing the ROS 1 bytes directly."""
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        for k, (_, _, raw) in enumerate(reader.messages(connections=conns)):
            if k % every:
                continue
            raw = bytes(raw)
            n_frame = struct.unpack_from("<I", raw, 12)[0]           # seq, sec, nsec, then frame_id string
            o = 16 + n_frame
            timebase = struct.unpack_from("<Q", raw, o)[0]
            o += 8 + 4 + 1 + 3                                        # timebase, point_num, lidar_id, rsvd[3]
            n = struct.unpack_from("<I", raw, o)[0]
            pts = np.frombuffer(raw, dtype=POINT, count=n, offset=o + 4)
            ok = (pts["tag"] & 0x0F) == 0                             # Livox tag: drop noise-flagged returns
            pts = pts[ok]
            xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
            yield (timebase + pts["t"].astype(np.uint64)) * 1e-9, xyz


class GTInterpolator:
    def __init__(self, gt_file: Path):
        g = np.loadtxt(gt_file)
        t, keep = np.unique(g[:, 0], return_index=True)
        self.t, self.p = t, g[keep, 1:4]
        self.slerp = Slerp(t, Rotation.from_quat(g[keep, 4:8]))

    def __call__(self, t: np.ndarray):
        ok = (t >= self.t[0]) & (t <= self.t[-1])
        p = np.column_stack([np.interp(t[ok], self.t, self.p[:, i]) for i in range(3)])
        return ok, self.slerp(t[ok]), p


def accumulate(seq: str, every: int = 1, body_base: np.ndarray | None = None, rmin: float = 0.5, rmax: float = 12.0,
               voxel: float = 0.02, time_offset: float = 0.0, static_bin_s: float = 10.0,
               static_min_bins: int = 0) -> np.ndarray:
    """Accumulate the scans; with static_min_bins > 0, keep only points whose 10 cm cell is hit in at least
    that many distinct `static_bin_s` time windows (removes people walking through the scene)."""
    cfg = load_sequences()
    root = ROOT / cfg["data_root"]
    gt = GTInterpolator(root / seq / "gt_tum.txt")
    body_lidar = (body_base if body_base is not None else body_T_base()) @ base_T_lidar()
    chunks, cell_bins = [], []
    for t, xyz in read_scans(root / f"{seq}.bag", cfg["topics"]["lidar3d"], every):
        rng = np.linalg.norm(xyz, axis=1)
        sel = (rng > rmin) & (rng < rmax)
        t, xyz = t[sel], xyz[sel]
        ok, rot, pos = gt(t + time_offset)
        pb = xyz[ok] @ body_lidar[:3, :3].T + body_lidar[:3, 3]      # LiDAR -> mocap body
        pw = rot.apply(pb) + pos                                     # body -> world, per point
        _, first = np.unique(np.floor(pw / voxel).astype(np.int64), axis=0, return_index=True)
        chunks.append(pw[first])
        if static_min_bins:
            cells = np.unique(np.floor(pw / 0.10).astype(np.int64), axis=0)
            tbin = np.full((len(cells), 1), int((t[0] - gt.t[0]) // static_bin_s), np.int64)
            cell_bins.append(np.hstack([cells, tbin]))
    pts = np.concatenate(chunks)
    _, first = np.unique(np.floor(pts / voxel).astype(np.int64), axis=0, return_index=True)
    pts = pts[first]
    if static_min_bins:
        cb = np.unique(np.concatenate(cell_bins), axis=0)             # (cell, time bin) pairs
        cells, n_bins = np.unique(cb[:, :3], axis=0, return_counts=True)
        static = {tuple(c) for c in cells[n_bins >= static_min_bins]}
        keep = np.array([tuple(c) in static for c in np.floor(pts / 0.10).astype(np.int64)])
        print(f"static filter: kept {keep.mean() * 100:.1f} % of points (cell hit in >= {static_min_bins} "
              f"of {int(cb[:, 3].max()) + 1} windows of {static_bin_s:.0f} s)")
        pts = pts[keep]
    return pts


def _consistency_cost(scans, gt: GTInterpolator, params: np.ndarray, min_dt: float = 3.0, k: int = 8) -> float:
    """Point-to-plane consistency between scans taken at different times (Levinson & Thrun 2010 style):
    for every point, distance to the plane fitted to its k nearest neighbours from scans >= min_dt s away.
    Robust mean (Cauchy, 3 cm)."""
    from scipy.spatial import cKDTree
    roll, pitch, yaw, x, y, dt = params
    bl = body_T_base(yaw, (x, y, 0.0), roll, pitch) @ base_T_lidar()
    pts, times = [], []
    for t, xyz in scans:
        ok, rot, pos = gt(t + dt)
        pts.append(rot.apply(xyz[ok] @ bl[:3, :3].T + bl[:3, 3]) + pos)
        times.append(t[ok])
    p, tt = np.concatenate(pts), np.concatenate(times)
    tree = cKDTree(p)
    _, nn = tree.query(p, k=4 * k)
    far = np.abs(tt[nn] - tt[:, None]) >= min_dt
    res = []
    for i in np.flatnonzero(far.sum(1) >= k)[::3]:
        q = p[nn[i][far[i]][:k]]
        c = q.mean(0)
        normal = np.linalg.svd(q - c)[2][-1]
        res.append(abs((p[i] - c) @ normal))
    r = np.asarray(res)
    return float(np.mean(np.log1p((r / 0.03) ** 2)))


def calibrate(seq: str, n_scans: int = 80, points_per_scan: int = 1500, x0=(0, 0, 180.0, 0, 0, 0)):
    """Estimate body->base (roll, pitch, yaw, x, y) and a LiDAR-mocap time offset by maximising map consistency.
    z is unobservable for planar motion and stays 0."""
    from scipy.optimize import minimize
    cfg = load_sequences()
    root = ROOT / cfg["data_root"]
    gt = GTInterpolator(root / seq / "gt_tum.txt")
    all_scans = list(read_scans(root / f"{seq}.bag", cfg["topics"]["lidar3d"], every=1))
    rng = np.random.default_rng(0)
    scans = []
    for i in np.linspace(0, len(all_scans) - 1, n_scans).astype(int):
        t, xyz = all_scans[i]
        r = np.linalg.norm(xyz, axis=1)
        sel = np.flatnonzero((r > 0.5) & (r < 12.0))
        sel = rng.choice(sel, min(points_per_scan, len(sel)), replace=False)
        scans.append((t[sel], xyz[sel]))
    f = lambda p: _consistency_cost(scans, gt, p)
    print(f"start cost {f(np.array(x0, float)):.4f}")
    res = minimize(f, np.array(x0, float), method="Nelder-Mead",
                   options={"initial_simplex": np.array(x0, float) + np.vstack([np.zeros(6), np.diag([2, 2, 10, 0.1, 0.1, 0.02])]),
                            "xatol": 1e-3, "fatol": 1e-5, "maxiter": 600})
    print(f"end cost {res.fun:.4f} after {res.nit} iterations")
    return res.x


def write_ply(path: Path, xyz: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(f"ply\nformat binary_little_endian 1.0\nelement vertex {len(xyz)}\n"
                "property float x\nproperty float y\nproperty float z\nend_header\n".encode())
        f.write(xyz.astype("<f4").tobytes())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sequence", required=True)
    ap.add_argument("--every", type=int, default=1, help="use every k-th scan (quick looks)")
    ap.add_argument("--calibrate", action="store_true", help="only estimate body->base + time offset and print them")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    if a.calibrate:
        print("roll, pitch, yaw [deg], x, y [m], time offset [s]:", [round(float(v), 4) for v in calibrate(a.sequence)])
        return
    cfg = load_sequences()
    rm = cfg["reference_map"]
    rig = rm["body_T_base"]
    bb = body_T_base(rig["yaw_deg"], (rig["x_m"], rig["y_m"], 0.0), rig["roll_deg"], rig["pitch_deg"])
    pts = accumulate(a.sequence, a.every, bb, voxel=rm["voxel_m"], time_offset=rm["time_offset_s"][a.sequence],
                     static_bin_s=rm["static_bin_s"], static_min_bins=rm["static_min_bins"])
    out = a.out or ROOT / cfg["data_root"] / a.sequence / "reference_map.ply"
    write_ply(out, pts)
    print(f"{a.sequence}: {len(pts)} points ({rm['voxel_m'] * 100:.0f} cm voxels) -> {out}")


if __name__ == "__main__":
    main()
