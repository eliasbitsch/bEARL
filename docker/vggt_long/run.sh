#!/usr/bin/env bash
# Run VGGT-Long headless on one image folder inside the container.
#
# Container mounts:
#   /data     image folder (*.jpg / *.png); VGGT-Long processes the files in sorted file-name order
#   /weights  folder with model.pt (VGGT-1B), dino_salad.ckpt, dinov2_vitb14_pretrain.pth
#   /out      output folder: camera_poses.txt (one 4x4 camera-to-world matrix per image, row-major),
#             combined_pcd.ply, the config used, log; also VGGT-Long's temporary files (deleted at the end)
#
# Environment (all optional; empty = upstream default from configs/base_config.yaml):
#   CHUNK_SIZE, OVERLAP, LOOP_CHUNK_SIZE   chunking (defaults 60 / 30 / 20; smaller only for GPU memory)
#   LOOP_ENABLE                            True / False
#   MODEL                                  VGGT (default) or Mapanything ("Map-Long", metric; weights in
#                                          /weights/map_anything = HF snapshot of facebook/map-anything)
#   USING_SIM3                             True / False chunk alignment; upstream: True for VGGT, False (SE(3))
#                                          for MapAnything ("mapanything need se3")
#   WEIGHTS_DTYPE=bf16                    keep the VGGT weights in bfloat16 on the GPU (upstream: float32 weights,
#                                          inference under bf16 autocast); saves ~2.5 GB for 16 GB cards
#   HOST_UID / HOST_GID                    chown the outputs to this user at the end
#
# Host example (WSL2):
#   docker run --rm --gpus all --ipc=host \
#     -v /path/to/images:/data:ro -v /path/to/weights:/weights:ro -v /path/to/out:/out \
#     -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) bearl/vggt_long
set -euo pipefail
cd /opt/VGGT-Long
# Apptainer images are read-only, but VGGT-Long writes next to its code (./exps link, optional patch)
if [ ! -w . ]; then
    cp -a /opt/VGGT-Long /tmp/VGGT-Long && cd /tmp/VGGT-Long
fi

python - <<'EOF'
import os, yaml
cfg = yaml.safe_load(open("configs/base_config.yaml"))
w = cfg["Weights"]
w["VGGT"] = "/weights/model.pt"
w["SALAD"] = "/weights/dino_salad.ckpt"
w["DNIO"] = "/weights/dinov2_vitb14_pretrain.pth"
w["Map"] = "/weights/map_anything"
if os.environ.get("MODEL"):
    w["model"] = os.environ["MODEL"]
m = cfg["Model"]
if os.environ.get("USING_SIM3"):
    m["using_sim3"] = os.environ["USING_SIM3"] == "True"
for env, key, typ in (("CHUNK_SIZE", "chunk_size", int), ("OVERLAP", "overlap", int),
                      ("LOOP_CHUNK_SIZE", "loop_chunk_size", int)):
    if os.environ.get(env):
        m[key] = typ(os.environ[env])
if os.environ.get("LOOP_ENABLE"):
    m["loop_enable"] = os.environ["LOOP_ENABLE"] == "True"
yaml.safe_dump(cfg, open("/out/config_used.yaml", "w"), sort_keys=False)
print("config:", {"model": w["model"], **{k: m[k] for k in ("chunk_size", "overlap", "loop_chunk_size", "loop_enable", "using_sim3")}})
EOF

if [ "${WEIGHTS_DTYPE:-}" = "bf16" ]; then
    # one line after the VGGT adapter moves the model to the GPU
    python - <<'EOF'
import re
p = "base_models/base_model.py"
src = open(p).read()
new, n = re.subn(r"^(\s*)(self\.model = self\.model\.to\(self\.device\))[ \t]*$",
                 r"\1\2\n\1self.model = self.model.to(torch.bfloat16)  # bEARL: WEIGHTS_DTYPE=bf16",
                 src, count=1, flags=re.M)
if n != 1:
    raise SystemExit("WEIGHTS_DTYPE=bf16: patch location not found in base_models/base_model.py")
open(p, "w").write(new)
print("bf16 weights: patched VGGT adapter")
EOF
fi

# VGGT-Long writes to ./exps/<image_dir>/<datetime>/ -> put that on the mounted output disk
mkdir -p /out/exps && rm -rf exps && ln -sfn /out/exps exps

echo "VGGT-Long commit: $(git rev-parse HEAD)"
python vggt_long.py --image_dir /data --config /out/config_used.yaml 2>&1 | tee /out/log.txt

RUN_DIR=$(ls -d /out/exps/*/* | tail -1)
cp "$RUN_DIR/camera_poses.txt" /out/camera_poses.txt
[ -f "$RUN_DIR/pcd/combined_pcd.ply" ] && cp "$RUN_DIR/pcd/combined_pcd.ply" /out/combined_pcd.ply
[ -f "$RUN_DIR/loop_closures.txt" ] && cp "$RUN_DIR/loop_closures.txt" /out/loop_closures.txt
echo "poses: $(wc -l < /out/camera_poses.txt)"

if [ -n "${HOST_UID:-}" ]; then
    chown -R "${HOST_UID}:${HOST_GID:-$HOST_UID}" /out
fi
