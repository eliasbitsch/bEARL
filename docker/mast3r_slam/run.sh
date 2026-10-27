#!/usr/bin/env bash
# Run MASt3R-SLAM headless on one image folder (Apptainer / Docker).
#
# Mounts: /data image folder (*.jpg or *.png), /ckpt the three MASt3R checkpoints, /out output folder.
# Output: /out/traj.txt (keyframe poses, TUM; timestamps = frame index / 30, as upstream's RGBFiles loader),
#         /out/map.ply, /out/log.txt, /out/config_used.yaml.
# Environment: CONFIG (default base = upstream default, uncalibrated), CALIB=/calib.yaml to pass intrinsics.
set -euo pipefail
cd /opt/MASt3R-SLAM
if [ ! -w . ]; then                       # Apptainer images are read-only; MASt3R-SLAM writes logs/ next to its code
    cp -a /opt/MASt3R-SLAM /tmp/MASt3R-SLAM && cd /tmp/MASt3R-SLAM
fi
ln -sfn /ckpt checkpoints

# upstream's image-folder loader only globs *.png; the M3DGR frames are stored as the original JPEGs
python - <<'EOF'
p = "mast3r_slam/dataloader.py"
src = open(p).read()
old = 'natsorted(list((self.dataset_path).glob("*.png")))'
if old not in src:
    raise SystemExit("RGBFiles glob not found in mast3r_slam/dataloader.py")
open(p, "w").write(src.replace(old, 'natsorted(list((self.dataset_path).glob("*.png")) + list((self.dataset_path).glob("*.jpg")))'))
print("RGBFiles: *.png and *.jpg")
EOF

CFG="config/${CONFIG:-base}.yaml"
cp "$CFG" /out/config_used.yaml
mkdir -p /out/logs && rm -rf logs && ln -sfn /out/logs logs
ARGS=(--dataset /data --config "$CFG" --save-as bearl --no-viz)
[ -n "${CALIB:-}" ] && ARGS+=(--calib "$CALIB")
echo "MASt3R-SLAM commit: $(git rev-parse HEAD 2>/dev/null || echo unknown)  frames: $(ls /data | wc -l)"
python main.py "${ARGS[@]}" 2>&1 | tee /out/log.txt

TRAJ=$(find /out/logs/bearl -maxdepth 1 -name "*.txt" | head -1)
PLY=$(find /out/logs/bearl -maxdepth 1 -name "*.ply" | head -1)
cp "$TRAJ" /out/traj.txt
[ -n "$PLY" ] && cp "$PLY" /out/map.ply
echo "keyframe poses: $(wc -l < /out/traj.txt)"
