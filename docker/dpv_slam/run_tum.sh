#!/usr/bin/env bash
# Run DPV-SLAM / DPVO on one TUM RGB-D sequence (monocular RGB) inside the container.
# Uses the upstream TUM pipeline from evaluate_tum.py (fr1 intrinsics, cv2.undistort, 16/8 px crop,
# stride 1, config/default.yaml, seed 1234) and writes the trajectory in TUM format with the real
# image timestamps (taken from the rgb/<timestamp>.png file names).
#
#   run_tum.sh <sequence_dir> <out_dir> <weights.pth> [config overrides ...]
#
# Examples (in the container):
#   run_tum.sh /data/rgbd_dataset_freiburg1_desk /out /weights/dpvo.pth LOOP_CLOSURE True   # DPV-SLAM
#   run_tum.sh /data/rgbd_dataset_freiburg1_desk /out /weights/dpvo.pth                     # DPVO (VO only)
#
# Host side (WSL), with the GPU lock and the measurement wrapper:
#   docker build -t bearl/dpv_slam /mnt/c/git/FH/bEARL/docker/dpv_slam
#   flock /tmp/bearl_gpu.lock python3 /mnt/c/git/FH/bEARL/tools/bench_gpu.py --name dpv_slam --frames 613 \
#     --out /home/elias/bEARL/results_bench/dpv_slam -- \
#     docker run --rm --gpus all --ipc=host \
#       -v /home/elias/bEARL/data/tum/rgbd_dataset_freiburg1_desk:/data/seq:ro \
#       -v /home/elias/bEARL/third_party/checkpoints/dpvo:/weights:ro \
#       -v /home/elias/bEARL/results_bench/dpv_slam:/out \
#       bearl/dpv_slam /opt/run_tum.sh /data/seq /out /weights/dpvo.pth LOOP_CLOSURE True
set -euo pipefail

SEQ_DIR=${1:?sequence dir}
OUT_DIR=${2:?output dir}
WEIGHTS=${3:?weights file}
shift 3

mkdir -p "$OUT_DIR"
cd /opt/DPVO

python - "$SEQ_DIR" "$OUT_DIR" "$WEIGHTS" "$@" <<'EOF'
import json, sys, time
from pathlib import Path

import numpy as np
import torch
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface

sys.path.insert(0, "/opt/DPVO")
from dpvo.config import cfg
from evaluate_tum import run

seq_dir, out_dir, weights = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
opts = sys.argv[4:]

cfg.merge_from_file("config/default.yaml")
cfg.BACKEND_THRESH = 64.0
cfg.merge_from_list(opts)
print(cfg, flush=True)
torch.manual_seed(1234)

t0 = time.perf_counter()
poses, tstamps = run(cfg, weights, seq_dir, seq_dir.name, stride=1, viz=False, show_img=False)
torch.cuda.synchronize()
dt = time.perf_counter() - t0
n_frames = len(list((seq_dir / "rgb").glob("*.png")))

traj = PoseTrajectory3D(positions_xyz=poses[:, :3], orientations_quat_wxyz=poses[:, [6, 3, 4, 5]],
                        timestamps=np.asarray(tstamps, dtype=np.float64))
file_interface.write_tum_trajectory_file(str(out_dir / "traj_tum.txt"), traj)

stats = {"frames": n_frames, "poses": int(len(tstamps)), "slam_time_s": round(dt, 2),
         "slam_fps": round(n_frames / dt, 2), "torch_peak_alloc_mib": round(torch.cuda.max_memory_allocated() / 2**20),
         "torch_peak_reserved_mib": round(torch.cuda.max_memory_reserved() / 2**20),
         "opts": opts, "gpu": torch.cuda.get_device_name(0)}
(out_dir / "run_stats.json").write_text(json.dumps(stats, indent=2))
print(json.dumps(stats, indent=2))
EOF
