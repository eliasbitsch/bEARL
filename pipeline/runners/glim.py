"""Runner for GLIM (Philip Stix).

GLIM (Koide et al., RAS 2024) runs in the authors' Docker image on the ROS 2 bag written by
tools/glim_prepare_bag.py (Livox CustomMsg -> PointCloud2, original timestamps):
    ros2 run glim_ros glim_rosbag <bag> --ros-args -p config_path:=... -p auto_quit:=true -p dump_path:=...
glim_rosbag throttles playback itself so that no scan is dropped (offline run).

GLIM writes its result to the dump directory (kept under data/, git-ignored, for the offline_viewer):
    traj_imu.txt / traj_lidar.txt    TUM trajectory with loop closure (IMU / LiDAR frame)
    odom_imu.txt / odom_lidar.txt    same without loop closure (odometry only)
    000000/ 000001/ ...              submaps: data.txt (T_world_origin) + points_compact.bin (float32 xyz)
The runner copies the configured trajectory to trajectory_tum.txt and merges the submaps into map.ply.

Note: GLIM runs inside the container, so resources.csv only sees the docker CLI process.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import numpy as np

from pipeline.common import ROOT


def run(sequence: dict, out_dir: Path, run_idx: int, cfg: dict, data_root: Path) -> None:
    """Run GLIM once on one sequence (GLIM has no random seed; run_idx only names the dump)."""
    g = cfg["glim"]
    bag = (data_root / sequence["name"] / "glim_ros2").resolve()
    if not (bag / "metadata.yaml").exists():
        raise FileNotFoundError(f"{bag} missing - run tools/glim_prepare_bag.py first")
    config_dir = (ROOT / g["config_dir"]).resolve()
    dump = (data_root / sequence["name"] / "glim_dumps" / f"run_{run_idx:02d}").resolve()
    if dump.exists():
        shutil.rmtree(dump)
    dump.parent.mkdir(parents=True, exist_ok=True)

    cmd = ["docker", "run", "--rm",
           "-v", f"{bag}:/data/bag:ro", "-v", f"{config_dir}:/glim/config:ro",
           "-v", f"{dump.parent}:/dumps"]
    if g.get("gpus", False):
        cmd += ["--gpus", "all"]
    cmd += [g["image"], "ros2", "run", "glim_ros", "glim_rosbag", "/data/bag", "--ros-args",
            "-p", "config_path:=/glim/config", "-p", "auto_quit:=true",
            "-p", f"dump_path:=/dumps/{dump.name}"]
    with open(out_dir / "glim.log", "w", encoding="utf-8") as log:
        subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True)

    traj = dump / g["trajectory"]
    if not traj.exists() or not traj.read_text().strip():
        raise RuntimeError(f"GLIM wrote no trajectory ({traj}) - see glim.log")
    shutil.copy(traj, out_dir / "trajectory_tum.txt")
    write_map_ply(dump, out_dir / "map.ply")


def write_map_ply(dump: Path, out_file: Path) -> int:
    """Merge all submaps of a GLIM dump into one point cloud in the map frame (binary PLY)."""
    clouds = []
    for sub in sorted(p for p in dump.iterdir() if p.is_dir() and p.name.isdigit()):
        T = _read_T_world_origin(sub / "data.txt")
        pts = np.fromfile(sub / "points_compact.bin", dtype="<f4").reshape(-1, 3).astype(np.float64)
        clouds.append(pts @ T[:3, :3].T + T[:3, 3])
    if not clouds:
        raise RuntimeError(f"no submaps in {dump}")
    pts = np.concatenate(clouds).astype("<f4")
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(pts)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
    with open(out_file, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts.tobytes())
    return len(pts)


def _read_T_world_origin(data_txt: Path) -> np.ndarray:
    lines = [line.strip() for line in data_txt.read_text().splitlines()]
    i = lines.index("T_world_origin:")
    return np.array([[float(v) for v in line.split()] for line in lines[i + 1:i + 5]])
