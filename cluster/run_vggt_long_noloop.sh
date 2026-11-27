#!/bin/bash
# Diagnostic (not a main result): VGGT-Long on Varying-illu01 with loop closure switched off, everything else at the
# authors' defaults. Tests whether loop pairs between different blackouts cause the collapse (review finding).
#   cd <repo> && sbatch cluster/run_vggt_long_noloop.sh                 # all frames
#   cd <repo> && MODE=lit sbatch --export=ALL cluster/run_vggt_long_noloop.sh   # dark frames removed, as run_vggt_long_lit_only.sh
#SBATCH --job-name=vggt-noloop
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
MODE=${MODE:-all}
if [ "$MODE" = lit ]; then
  IMG="$SLURM_SUBMIT_DIR/data/M3DGR/$SEQ/rgb_step3_lit"
  OUT="$SLURM_SUBMIT_DIR/results/vggt_long_lit_noloop/$SEQ/run_00"
  mkdir -p "$IMG"
  while read -r f; do ln -sf "../rgb_step3/$f" "$IMG/$f"; done < cluster/lit_frames_varying.txt
else
  IMG="$SLURM_SUBMIT_DIR/data/M3DGR/$SEQ/rgb_step3"
  OUT="$SLURM_SUBMIT_DIR/results/vggt_long_noloop/$SEQ/run_00"
fi
mkdir -p "$OUT/vggt_long"
echo "mode $MODE, frames: $(ls "$IMG" | wc -l)"
cp docker/vggt_long/run.sh "$OUT/vggt_long/run.sh"; chmod +x "$OUT/vggt_long/run.sh"
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
apptainer run --nv \
  --bind "$IMG:/data:ro,$HOME/bearl/weights/vggt_long:/weights:ro,$OUT/vggt_long:/out,$OUT/vggt_long/run.sh:/opt/run.sh:ro,$SLURM_SUBMIT_DIR/data/M3DGR/$SEQ/rgb_step3:/rgb_step3:ro" \
  --env XFORMERS_DISABLED=1 --env LOOP_ENABLE=False \
  containers/vggt_long.sif
rm -rf "$OUT/vggt_long/exps"
ls -la "$OUT/vggt_long"
