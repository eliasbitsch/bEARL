#!/bin/bash
# VGGT-Long on the ADA cluster: one array task per (sequence, run), 48 GB GPU (MIG 2g.48gb), not preemptible.
#   cd <repo> && sbatch cluster/run_vggt_long.sh            # all 3 sequences x 5 runs
#   sbatch --array=0 cluster/run_vggt_long.sh               # only Dynamic01, run 0 (smoke test)
# Needs: containers/vggt_long.sif (cluster/build_vggt_long.sh), weights in $VGGT_LONG_WEIGHTS,
#        data/M3DGR/<seq>/{rgb_step3/, gt_tum.txt}, and a Python env with numpy, scipy, pyyaml, psutil.
#SBATCH --job-name=vggt-long
#SBATCH --partition=shared
#SBATCH --constraint=gpu_vram_ge_48
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=48G
#SBATCH --time=03:00:00
#SBATCH --array=0-14
#SBATCH --output=logs/%x_%A_%a.out

set -euo pipefail
SEQUENCES=(Dynamic01 Sha-turn01 Varying-illu01)
RUNS_PER_SEQ=5
SEQ=${SEQUENCES[$((SLURM_ARRAY_TASK_ID / RUNS_PER_SEQ))]}
RUN=$((SLURM_ARRAY_TASK_ID % RUNS_PER_SEQ))

cd "$SLURM_SUBMIT_DIR"
module load SciPy-bundle/2024.05-gfbf-2024a
source "$HOME/bearl/venv/bin/activate"
export VGGT_LONG_LAUNCHER=apptainer
export VGGT_LONG_SIF="$SLURM_SUBMIT_DIR/containers/vggt_long.sif"
export VGGT_LONG_WEIGHTS="${VGGT_LONG_WEIGHTS:-$HOME/bearl/weights/vggt_long}"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
python -m pipeline.run_all --system vggt_long --sequence "$SEQ" --run "$RUN"
