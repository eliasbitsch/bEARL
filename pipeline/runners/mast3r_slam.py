"""Runner for MASt3R-SLAM (Elias Bitsch, RGB deep dive).

MASt3R-SLAM (Murai et al., CVPR 2025) in the container docker/mast3r_slam (Apptainer on the cluster:
mast3r_slam.def). Input: all frames of <sequence>/rgb/ (30 Hz, upstream default). Output: keyframe poses, as in
the authors' evaluation; upstream stamps frames with index / 30, mapped back to the image timestamps here.

Environment:
    MAST3R_SLAM_CKPT      folder with the three MASt3R checkpoints (default: third_party/weights/mast3r)
    MAST3R_SLAM_LAUNCHER  docker (default; through `wsl.exe docker` on Windows) or apptainer
    MAST3R_SLAM_SIF       Apptainer image (default: mast3r_slam.sif)
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import numpy as np

from pipeline.common import ROOT
from pipeline.runners.vggt_long import _host, _subsampled_images


def _container_cmd(image_dir: Path, ckpt: Path, work: Path, params: dict) -> list[str]:
    env = {"CONFIG": params.get("config")}
    env = {k: str(v) for k, v in env.items() if v is not None}
    if os.environ.get("MAST3R_SLAM_LAUNCHER", "docker") == "apptainer":
        run_sh = ROOT / "docker" / "mast3r_slam" / "run.sh"
        shutil.copy(run_sh, work / "run.sh")        # per-run copy: an edit during a running job must not reach it
        binds = f"{image_dir}:/data:ro,{ckpt}:/ckpt:ro,{work}:/out,{work / 'run.sh'}:/opt/run.sh:ro"
        envs = [a for k, v in env.items() for a in ("--env", f"{k}={v}")]
        return ["apptainer", "run", "--nv", "--bind", binds, *envs, os.environ.get("MAST3R_SLAM_SIF", "mast3r_slam.sif")]
    cmd = ["docker", "run", "--rm", "--gpus", "all", "--ipc=host", "-v", f"{_host(image_dir)}:/data:ro",
           "-v", f"{_host(ckpt)}:/ckpt:ro", "-v", f"{_host(work)}:/out"]
    cmd += [a for k, v in env.items() for a in ("-e", f"{k}={v}")] + ["bearl/mast3r_slam"]
    if os.name == "nt":
        cmd = ["wsl.exe", "-d", os.environ.get("WSL_DISTRO", "Ubuntu-24.04"), "--", *cmd]
    return cmd


def to_tum(traj: Path, image_dir: Path, out_file: Path) -> int:
    """Upstream time = frame index / 30 -> timestamp of that frame (file names are the image timestamps)."""
    frames = sorted(image_dir.glob("*.jpg")) + sorted(image_dir.glob("*.png"))
    stamps = np.array([float(f.stem) for f in frames])
    rows = np.loadtxt(traj, ndmin=2)
    idx = np.rint(rows[:, 0] * 30).astype(int)
    rows[:, 0] = stamps[idx]
    np.savetxt(out_file, rows, fmt="%.6f")
    return len(rows)


def run(sequence: dict, out_dir: Path, run_idx: int, cfg: dict, data_root: Path) -> None:
    params = cfg.get("parameters", {})
    image_dir = _subsampled_images(data_root / sequence["name"], int(params.get("frame_step", 1)))
    ckpt = Path(os.environ.get("MAST3R_SLAM_CKPT", ROOT / "third_party" / "weights" / "mast3r"))
    work = out_dir / "mast3r_slam"
    work.mkdir(exist_ok=True)
    cmd = _container_cmd(image_dir, ckpt, work, params)
    (out_dir / "command.txt").write_text(" ".join(cmd) + "\n")
    subprocess.run(cmd, check=True)
    n = to_tum(work / "traj.txt", image_dir, out_dir / "trajectory_tum.txt")
    if (work / "map.ply").exists():
        shutil.move(work / "map.ply", out_dir / "map.ply")
    shutil.rmtree(work / "logs", ignore_errors=True)
    print(f"MASt3R-SLAM {sequence['name']} run {run_idx}: {n} keyframe poses")
