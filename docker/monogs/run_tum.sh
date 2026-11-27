#!/usr/bin/env bash
# Run MonoGS headless in monocular mode with --eval on one TUM RGB-D sequence inside the container,
# then export the estimated trajectories in TUM format with the original image timestamps.
#
# Container mounts:
#   /opt/MonoGS/datasets/tum/<SEQ>  TUM sequence folder (rgb/, rgb.txt, depth.txt, groundtruth.txt), read-only;
#                                   this is the path the upstream configs/mono/tum/*.yaml expect
#   /out                            output folder
#
# Environment:
#   CONFIG=configs/mono/tum/fr1_desk.yaml   MonoGS config
#   SEQ=rgbd_dataset_freiburg1_desk         sequence folder name (must match the config's dataset_path)
#   HOST_UID/HOST_GID                       chown the outputs to this user at the end
#
# Outputs in /out:
#   monogs_results/...          MonoGS result folder (config, plots, trj_final.json, psnr/ jsons, point clouds)
#   traj_tum.txt                keyframe trajectory (what MonoGS itself evaluates), TUM format
#   traj_tum_allframes.txt      pose of every tracked frame, TUM format
#
# Host example (WSL2):
#   docker run --rm --gpus all --ipc=host \
#     -v /home/elias/bEARL/data/tum/rgbd_dataset_freiburg1_desk:/opt/MonoGS/datasets/tum/rgbd_dataset_freiburg1_desk:ro \
#     -v /home/elias/bEARL/results_bench/monogs:/out \
#     -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) bearl/monogs
set -euo pipefail

CONFIG="${CONFIG:-configs/mono/tum/fr1_desk.yaml}"
SEQ="${SEQ:-rgbd_dataset_freiburg1_desk}"
DATASET="/opt/MonoGS/datasets/tum/${SEQ}"

cd /opt/MonoGS
mkdir -p /out/monogs_results
rm -rf results && ln -sfn /out/monogs_results results

echo "MonoGS commit: $(git rev-parse HEAD)  branch: main  config: $CONFIG"
rc=0
python slam.py --config "$CONFIG" --eval || rc=$?

RUN_DIR="$(ls -td results/tum_${SEQ}/*/ 2>/dev/null | head -1 || true)"
if [ -n "$RUN_DIR" ]; then
    echo "result folder: $RUN_DIR"
    python3 - "$DATASET" "$RUN_DIR" /out <<'EOF'
import json, os, sys
import numpy as np
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
from utils.dataset import TUMParser

dataset, run_dir, out = sys.argv[1:4]
parser = TUMParser(dataset)
stamps = [float(os.path.splitext(os.path.basename(p))[0]) for p in parser.color_paths]
print("frames used by MonoGS:", len(stamps))

def write(ids, poses, name):
    order = np.argsort([stamps[i] for i in ids])
    traj = PoseTrajectory3D(poses_se3=[poses[i] for i in order],
                            timestamps=np.array([stamps[ids[i]] for i in order]))
    file_interface.write_tum_trajectory_file(os.path.join(out, name), traj)
    print(name, "poses:", len(order))

kf = os.path.join(run_dir, "plot", "trj_final.json")
if os.path.isfile(kf):
    d = json.load(open(kf))
    write(d["trj_id"], [np.array(p) for p in d["trj_est"]], "traj_tum.txt")

allf = os.path.join(run_dir, "trj_all_frames.txt")
if os.path.isfile(allf):
    ids, poses = [], []
    for line in open(allf):
        v = line.split()
        T = np.eye(4)
        T[:3, :] = np.array(v[1:], dtype=float).reshape(3, 4)
        ids.append(int(v[0])); poses.append(T)
    write(ids, poses, "traj_tum_allframes.txt")
EOF
fi

if [ -n "${HOST_UID:-}" ]; then
    chown -R "${HOST_UID}:${HOST_GID:-$HOST_UID}" /out
fi
exit "$rc"
