#!/bin/bash
# Diagnostic (not a main result): VGGT-Long with the authors' defaults on Varying-illu01, near-black frames removed
# from the input (list from tools/build_cache.py brightness, > 80 % pixels below 20). Tests whether the dark chunks
# cause the collapse. Algorithm and parameters unchanged.
#   cd <repo> && sbatch cluster/run_vggt_long_lit_only.sh
#SBATCH --job-name=vggt-lit
#SBATCH --partition=shared
#SBATCH --constraint=gpu_vram_ge_48
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=48G
#SBATCH --time=02:00:00
#SBATCH --output=logs/%x_%j.out
set -euo pipefail
cd "$SLURM_SUBMIT_DIR"
SEQ=Varying-illu01
IMG="$SLURM_SUBMIT_DIR/data/M3DGR/$SEQ/rgb_step3_lit"
OUT="$SLURM_SUBMIT_DIR/results/vggt_long_lit_only/$SEQ/run_00"
mkdir -p "$IMG" "$OUT/vggt_long"
while read -r f; do ln -sf "../rgb_step3/$f" "$IMG/$f"; done < cluster/lit_frames_varying.txt
echo "lit frames: $(ls "$IMG" | wc -l)"
cp docker/vggt_long/run.sh "$OUT/vggt_long/run.sh"; chmod +x "$OUT/vggt_long/run.sh"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
apptainer run --nv \
  --bind "$IMG:/data:ro,$HOME/bearl/weights/vggt_long:/weights:ro,$OUT/vggt_long:/out,$OUT/vggt_long/run.sh:/opt/run.sh:ro,$SLURM_SUBMIT_DIR/data/M3DGR/$SEQ/rgb_step3:/rgb_step3:ro" \
  --env XFORMERS_DISABLED=1 \
  containers/vggt_long.sif
rm -rf "$OUT/vggt_long/exps"
ls -la "$OUT/vggt_long"
