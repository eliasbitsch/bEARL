"""Runner for FAST-LIO2 (Xu et al., T-RO 2022) in the version adapted by the M3DGR authors.

Container docker/fast_lio2 replays the original ROS 1 bag in real time through FAST-LIO2 with the MID-360 config of
M3DGR's baseline_systems/Fast_LIO2_M3DGR, and records /Odometry. Output conversion:
    odom.csv (rostopic echo -p /Odometry) -> trajectory_tum.txt, pose of the MID-360 IMU in the first IMU frame
    scans.pcd (all registered scans, written on shutdown) -> map.ply, same frame as the trajectory
"""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

import numpy as np
import pandas as pd

from pipeline.common import ROOT
from pipeline.runners.vggt_long import _host


def odom_csv_to_tum(csv: Path, out: Path) -> int:
    df = pd.read_csv(csv)
    p, q = "field.pose.pose.position.", "field.pose.pose.orientation."
    tum = np.column_stack([df["field.header.stamp"].to_numpy() * 1e-9,
                           df[[p + "x", p + "y", p + "z"]].to_numpy(),
                           df[[q + "x", q + "y", q + "z", q + "w"]].to_numpy()])
    np.savetxt(out, tum, fmt="%.9f")
    return len(tum)


def pcd_to_ply(pcd: Path, ply: Path) -> None:
    import open3d as o3d
    cloud = o3d.io.read_point_cloud(str(pcd))
    o3d.io.write_point_cloud(str(ply), cloud.voxel_down_sample(0.02))     # same 2 cm grid as map_eval


def run(sequence: dict, out_dir: Path, run_idx: int, cfg: dict, data_root: Path) -> None:
    """Real-time bag replay; FAST-LIO2 is not deterministic across runs (threading, message timing)."""
    work = out_dir / "fast_lio2"
    work.mkdir(exist_ok=True)
    cmd = ["docker", "run", "--rm", "-v", f"{_host(data_root)}:/data:ro", "-v", f"{_host(work)}:/out",
           "-e", f"SEQ={sequence['name']}", "-e", f"RATE={cfg.get('parameters', {}).get('rate', 1.0)}"]
    if os.name != "nt":
        cmd += ["-e", f"HOST_UID={os.getuid()}", "-e", f"HOST_GID={os.getgid()}"]
    cmd += ["bearl/fast_lio2"]
    if os.name == "nt":
        cmd = ["wsl.exe", "-d", os.environ.get("WSL_DISTRO", "Ubuntu-24.04"), "--", *cmd]
    (out_dir / "command.txt").write_text(" ".join(cmd).replace(str(ROOT), "<repo>") + "\n")
    subprocess.run(cmd, check=True)

    n = odom_csv_to_tum(work / "odom.csv", out_dir / "trajectory_tum.txt")
    if (work / "scans.pcd").exists():
        pcd_to_ply(work / "scans.pcd", out_dir / "map.ply")
        (work / "scans.pcd").unlink()
    print(f"FAST-LIO2 {sequence['name']} run {run_idx}: {n} poses")
