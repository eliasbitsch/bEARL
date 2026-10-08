#!/bin/bash
# Batch runs of one GPU-based system on a Slurm cluster: one array task per (sequence, run).
#   sbatch --export=SLUG=mast3r_slam cluster/slurm_array.sh
# Adapt partition / account / container to the cluster's rules before the first submission.
#SBATCH --job-name=slam-eval
#SBATCH --array=0-14               # 3 sequences x 5 runs
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=02:00:00
#SBATCH --output=logs/%x_%A_%a.out

set -euo pipefail
SEQUENCES=(Varying-illu01 Dynamic01 Dark03)
RUNS_PER_SEQ=5

SEQ=${SEQUENCES[$((SLURM_ARRAY_TASK_ID / RUNS_PER_SEQ))]}
RUN=$((SLURM_ARRAY_TASK_ID % RUNS_PER_SEQ))

cd "$SLURM_SUBMIT_DIR"
# If the cluster has no Docker, build an Apptainer image from the system's Dockerfile:
#   apptainer build ${SLUG}.sif docker-daemon://${SLUG}:latest
# and run:  apptainer exec --nv ${SLUG}.sif python -m pipeline.run_all ...
python -m pipeline.run_all --system "$SLUG" --sequence "$SEQ" --run "$RUN"
