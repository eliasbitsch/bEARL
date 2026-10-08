"""Runner for MASt3R-SLAM (Elias Bitsch).

MASt3R-SLAM (Murai et al., CVPR 2025) runs on a folder of monocular RGB images:
    python main.py --dataset <image_folder> --config config/base.yaml [--calib intrinsics.yaml]
Set MAST3R_SLAM_DIR to the cloned upstream repository (pinned commit, see config/systems/mast3r_slam.yaml).

Input prepared by pipeline/prepare_data.py:
    <data_root>/<sequence>/rgb/<timestamp>.png   (one file per frame)
    <data_root>/<sequence>/rgb.txt               ("timestamp filename" per line, TUM RGB-D style)
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


def run(sequence: dict, out_dir: Path, run_idx: int, cfg: dict, data_root: Path) -> None:
    repo = Path(os.environ.get("MAST3R_SLAM_DIR", "third_party/MASt3R-SLAM")).resolve()
    seq_dir = data_root / sequence["name"]
    image_dir = seq_dir / "rgb"
    if not image_dir.is_dir():
        raise FileNotFoundError(f"{image_dir} missing - run pipeline/prepare_data.py first")

    cmd = [sys.executable, "main.py", "--dataset", str(image_dir.resolve()),
           "--config", "config/base.yaml", "--save-as", f"{sequence['name']}_run{run_idx:02d}",
           "--no-viz"]
    calib = seq_dir / "intrinsics_mast3r.yaml"
    if calib.exists():
        cmd += ["--calib", str(calib.resolve())]
    subprocess.run(cmd, cwd=repo, check=True)

    # TODO (Elias): check the upstream output names/format at the pinned commit. MASt3R-SLAM
    # writes the keyframe trajectory and the reconstruction to logs/. Its timestamps for an
    # image folder are frame indices -> map them back to rgb.txt timestamps here.
    log_dir = repo / "logs" / f"{sequence['name']}_run{run_idx:02d}"
    _to_tum_with_timestamps(log_dir / f"{image_dir.name}.txt", seq_dir / "rgb.txt",
                            out_dir / "trajectory_tum.txt")
    shutil.copy(log_dir / f"{image_dir.name}.ply", out_dir / "map.ply")


def _to_tum_with_timestamps(traj_file: Path, rgb_list: Path, out_file: Path) -> None:
    """Replace frame indices in the first column by the original image timestamps."""
    stamps = [line.split()[0] for line in rgb_list.read_text().splitlines()
              if line and not line.startswith("#")]
    lines = []
    for line in traj_file.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        idx, *pose = line.split()
        lines.append(" ".join([stamps[int(float(idx))], *pose]))
    out_file.write_text("\n".join(lines) + "\n")
