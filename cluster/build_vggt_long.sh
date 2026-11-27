#!/bin/bash
# Build an Apptainer image on a CPU node (no GPU needed; CUDA extensions are compiled for the
# architectures in TORCH_CUDA_ARCH_LIST of the .def file).
#   cd <repo> && sbatch cluster/build_vggt_long.sh                       # docker/vggt_long/vggt_long.def
#   NAME=mast3r_slam sbatch --export=ALL cluster/build_vggt_long.sh      # docker/mast3r_slam/mast3r_slam.def
#SBATCH --job-name=build-vggt-long
#SBATCH --partition=shared
#SBATCH --cpus-per-task=8
#SBATCH --mem=96G
#SBATCH --time=04:00:00
#SBATCH --output=logs/%x_%j.out

set -euo pipefail
cd "$SLURM_SUBMIT_DIR"
export APPTAINER_CACHEDIR="$HOME/.apptainer/cache"
# Build sandbox in /tmp: on ADA that is RAM (tmpfs), so it counts against --mem. On the GPFS home,
# extracting the ~10 GB PyTorch base image alone took > 2 h; the first build hit the time limit.
export APPTAINER_TMPDIR="/tmp/apptainer-$SLURM_JOB_ID"
mkdir -p "$APPTAINER_CACHEDIR" "$APPTAINER_TMPDIR" containers
NAME="${NAME:-vggt_long}"
cd "docker/$NAME"            # %files paths in the .def are relative to this folder
apptainer build --fakeroot --ignore-fakeroot-command --force \
    "$SLURM_SUBMIT_DIR/containers/$NAME.sif" "$NAME.def"
ls -la "$SLURM_SUBMIT_DIR/containers/$NAME.sif"
