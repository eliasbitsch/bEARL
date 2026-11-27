#!/usr/bin/env bash
# Usage: run_tum.sh <mono|rgbd> [--slam]
# Builds the image if needed, runs PyCuVSLAM on TUM fr1/desk under the GPU lock and
# the benchmark wrapper, then evaluates with evo_ape.
set -euo pipefail
MODE=${1:-mono}; shift || true
EXTRA=("$@")
REPO=$(cd "$(dirname "$0")/../.." && pwd)
IMAGE=bearl/cuvslam:17.0.0
DATA=${DATA:-/home/elias/bEARL/data/tum/rgbd_dataset_freiburg1_desk}
OUT=${OUT:-/home/elias/bEARL/results_bench/cuvslam_${MODE}}
EVO=${EVO:-/home/elias/.venvs/bearl/bin}

WHEELS=${WHEELS:-/home/elias/bEARL/wheels/cuvslam}
WHEEL=cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_x86_64.whl
if [ ! -f "$WHEELS/$WHEEL" ]; then
  mkdir -p "$WHEELS"
  curl -fL --retry 8 -C - -o "$WHEELS/$WHEEL" \
    "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/${WHEEL/+/%2B}"
fi
# --network=host: downloads over the default bridge kept getting reset under WSL2
docker build --network=host --build-context wheels="$WHEELS" -t "$IMAGE" "$REPO/docker/cuvslam"
mkdir -p "$OUT"
NFRAMES=$(grep -vc '^#' "$DATA/rgb.txt")

flock /tmp/bearl_gpu.lock python3 "$REPO/tools/bench_gpu.py" \
  --name "cuvslam_${MODE}" --frames "$NFRAMES" --out "$OUT" -- \
  docker run --rm --gpus all --ipc=host --user "$(id -u):$(id -g)" \
    -v "$DATA:/data:ro" -v "$OUT:/out" "$IMAGE" \
    --dataset /data --out /out --mode "$MODE" "${EXTRA[@]}"

if [ "$MODE" = mono ]; then ALIGN=-as; else ALIGN=-a; fi
"$EVO/evo_ape" tum "$DATA/groundtruth.txt" "$OUT/traj_tum.txt" $ALIGN --t_max_diff 0.02 \
  | tee "$OUT/evo_ape.txt"
