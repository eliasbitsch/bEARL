"""Runner for VGGT-Long (Elias Bitsch).

VGGT-Long (Deng et al., ICRA 2026) runs offline on a folder of monocular RGB images, in the
container docker/vggt_long (see the Dockerfile for the pinned upstream commit):
    docker run --gpus all -v <images>:/data:ro -v <weights>:/weights:ro -v <out>:/out bearl/vggt_long

Input prepared by pipeline/prepare_data.py:
    <data_root>/<sequence>/rgb/<timestamp>.jpg   (one file per frame, 30 Hz)
The runner links every `frame_step`-th frame into <sequence>/rgb_step<k>/ (see config/systems/vggt_long.yaml).

Output of VGGT-Long: camera_poses.txt, one 4x4 camera-to-world matrix (row-major) per input image, in
sorted file-name order -> converted to TUM with the image timestamps; pcd/combined_pcd.ply -> map.ply.

Environment:
    VGGT_LONG_WEIGHTS   folder with model.pt, dino_salad.ckpt, dinov2_vitb14_pretrain.pth
                        (default: third_party/weights/vggt_long)
    VGGT_LONG_LAUNCHER  docker (default; on Windows it runs through `wsl.exe docker`) or apptainer
    VGGT_LONG_SIF       Apptainer image for the cluster (default: vggt_long.sif)
    WSL_DISTRO          WSL distribution with Docker + GPU (default: Ubuntu-24.04)
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from pipeline.common import ROOT, load_sequences


def _wsl_path(p: Path) -> str:
    p = p.resolve()
    return f"/mnt/{p.drive[0].lower()}{p.as_posix()[2:]}"


def _host(p: Path) -> str:
    return _wsl_path(p) if os.name == "nt" else str(p.resolve())


def _subsampled_images(seq_dir: Path, step: int) -> Path:
    """Folder with every `step`-th frame of rgb/ as hard links (no copies, built once)."""
    src = seq_dir / "rgb"
    dst = seq_dir / f"rgb_step{step}"
    frames = sorted(src.glob("*.jpg")) + sorted(src.glob("*.png"))
    if not frames:
        if step > 1 and dst.is_dir() and any(dst.iterdir()):
            return dst                      # e.g. on the cluster, where only rgb_step<k>/ is uploaded
        raise FileNotFoundError(f"{src} is empty - run pipeline/prepare_data.py first")
    if step == 1:
        return src
    wanted = frames[::step]
    if dst.is_dir() and len(list(dst.iterdir())) == len(wanted):
        return dst
    shutil.rmtree(dst, ignore_errors=True)
    dst.mkdir()
    for f in wanted:
        os.link(f, dst / f.name)
    return dst


def _container_cmd(image_dir: Path, weights: Path, work: Path, params: dict, calib: Path | None = None) -> list[str]:
    env = {"CHUNK_SIZE": params.get("chunk_size"), "OVERLAP": params.get("overlap"),
           "LOOP_CHUNK_SIZE": params.get("loop_chunk_size"), "LOOP_ENABLE": params.get("loop_enable"),
           "MODEL": params.get("model"), "USING_SIM3": params.get("using_sim3"),
           "CALIB": True if calib else None}
    if calib:
        k = load_sequences()["rgb_intrinsics"]
        env.update(CALIB_W=k["w"], CALIB_H=k["h"])
    env = {k: str(v) for k, v in env.items() if v is not None}
    launcher = os.environ.get("VGGT_LONG_LAUNCHER", "docker")
    if launcher == "apptainer":
        # ADA has Blackwell GPUs (SM 12.0): xformers' FlashAttention-3 kernels fail there ("invalid argument").
        # xformers is only used by DINOv2 in the loop detector; this switch makes DINOv2 use plain (exact)
        # PyTorch attention instead. Also set in vggt_long.def for future builds.
        env["XFORMERS_DISABLED"] = "1"
        binds = f"{image_dir}:/data:ro,{weights}:/weights:ro,{work}:/out"
        # repo version of run.sh wins over the copy baked into the image; a per-run copy, because bash reads scripts
        # incrementally and an edit of the repo file during a running job corrupts it
        run_sh = ROOT / "docker" / "vggt_long" / "run.sh"
        if run_sh.exists():
            shutil.copy(run_sh, work / "run.sh")
            binds += f",{work / 'run.sh'}:/opt/run.sh:ro"
        if calib:
            binds += f",{calib}:/calib.txt:ro"
        envs = [a for k, v in env.items() for a in ("--env", f"{k}={v}")]
        return ["apptainer", "run", "--nv", "--bind", binds, *envs, os.environ.get("VGGT_LONG_SIF", "vggt_long.sif")]
    cmd = ["docker", "run", "--rm", "--gpus", "all", "--ipc=host",
           "-v", f"{_host(image_dir)}:/data:ro", "-v", f"{_host(weights)}:/weights:ro", "-v", f"{_host(work)}:/out"]
    if calib:
        cmd += ["-v", f"{_host(calib)}:/calib.txt:ro"]
    cmd += [a for k, v in env.items() for a in ("-e", f"{k}={v}")]
    if os.name != "nt":
        cmd += ["-e", f"HOST_UID={os.getuid()}", "-e", f"HOST_GID={os.getgid()}"]
    cmd += ["bearl/vggt_long"]
    if os.name == "nt":
        cmd = ["wsl.exe", "-d", os.environ.get("WSL_DISTRO", "Ubuntu-24.04"), "--", *cmd]
    return cmd


def _write_kitti_calib(path: Path) -> Path:
    """VGGT-Long reads K from a KITTI-style `P2:` line in <image_dir>/../calib.txt (mounted as /calib.txt)."""
    k = load_sequences()["rgb_intrinsics"]
    path.write_text(f"P2: {k['fx']} 0 {k['cx']} 0 0 {k['fy']} {k['cy']} 0 0 0 1 0\n")
    return path


def poses_to_tum(poses_file: Path, image_dir: Path, out_file: Path) -> int:
    """camera_poses.txt (4x4 c2w per sorted image) -> TUM 't x y z qx qy qz qw' with image timestamps."""
    frames = sorted(image_dir.glob("*.jpg")) + sorted(image_dir.glob("*.png"))
    poses = np.loadtxt(poses_file).reshape(-1, 4, 4)
    if len(poses) != len(frames):
        raise RuntimeError(f"{len(poses)} poses for {len(frames)} images")
    stamps = np.array([float(f.stem) for f in frames])
    q = Rotation.from_matrix(poses[:, :3, :3]).as_quat()          # x y z w
    tum = np.column_stack([stamps, poses[:, :3, 3], q])
    np.savetxt(out_file, tum, fmt="%.6f")
    return len(tum)


def run(sequence: dict, out_dir: Path, run_idx: int, cfg: dict, data_root: Path) -> None:
    """Run VGGT-Long once on one sequence (deterministic inference; run_idx is not used as a seed)."""
    params = cfg.get("parameters", {})
    image_dir = _subsampled_images(data_root / sequence["name"], int(params.get("frame_step", 1)))
    weights = Path(os.environ.get("VGGT_LONG_WEIGHTS", ROOT / "third_party" / "weights" / "vggt_long"))
    work = out_dir / "vggt_long"
    work.mkdir(exist_ok=True)

    calib = None
    if params.get("calib"):                # MapAnything only: real intrinsics instead of its own estimate
        calib = _write_kitti_calib(work / "calib.txt")
    cmd = _container_cmd(image_dir, weights, work, params, calib)
    (out_dir / "command.txt").write_text(" ".join(cmd) + "\n")
    subprocess.run(cmd, check=True)

    n = poses_to_tum(work / "camera_poses.txt", image_dir, out_dir / "trajectory_tum.txt")
    if (work / "combined_pcd.ply").exists():
        shutil.move(work / "combined_pcd.ply", out_dir / "map.ply")
    shutil.rmtree(work / "exps", ignore_errors=True)              # per-chunk point clouds, large
    print(f"VGGT-Long {sequence['name']} run {run_idx}: {n} poses")
