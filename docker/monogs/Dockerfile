# MonoGS / Gaussian Splatting SLAM (Matsuki, Murai, Kelly, Davison, CVPR 2024), headless build for benchmarking.
#
# Upstream pins torch 1.12.1 + cudatoolkit 11.6 (conda, python 3.7). This image uses the same torch
# version from the official cu116 pip wheels on a CUDA 11.6 devel base (system python 3.8), so the
# CUDA extensions (diff-gaussian-rasterization-w-pose, simple-knn) compile with a matching nvcc.
#
# Build:
#   docker build -t bearl/monogs docker/monogs
# Run: see run_tum.sh (dataset and output folder are mounted at runtime).
FROM nvidia/cuda:11.6.2-devel-ubuntu20.04

ARG MONOGS_COMMIT=6c9254c319d8bff5caeef65259e6bb0941a9b9f6

ENV DEBIAN_FRONTEND=noninteractive \
    TORCH_CUDA_ARCH_LIST="8.6" \
    MAX_JOBS=8 \
    PYTHONUNBUFFERED=1 \
    WANDB_MODE=disabled \
    MPLBACKEND=Agg

RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates build-essential ninja-build \
        python3.8 python3.8-dev python3-pip python-is-python3 \
        libgl1 libegl1 libgomp1 libglib2.0-0 libsm6 libxext6 libxrender1 libx11-6 libglfw3 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade "pip==23.3.2" "setuptools==69.0.3" wheel \
    && printf 'torch==1.12.1+cu116\ntorchvision==0.13.1+cu116\nnumpy==1.23.5\n' > /etc/pip-constraints.txt \
    && pip install --no-cache-dir -c /etc/pip-constraints.txt \
        torch==1.12.1+cu116 torchvision==0.13.1+cu116 torchaudio==0.12.1+cu116 numpy==1.23.5 \
        --extra-index-url https://download.pytorch.org/whl/cu116

ENV PIP_CONSTRAINT=/etc/pip-constraints.txt

RUN pip install --no-cache-dir \
        plyfile==0.8.1 tqdm opencv-python-headless==4.8.1.78 munch trimesh evo==1.11.0 open3d==0.17.0 \
        torchmetrics==1.0.3 imgviz PyOpenGL glfw PyGLM wandb==0.15.12 lpips==0.1.4 rich

WORKDIR /opt
RUN git clone --recursive https://github.com/muskie82/MonoGS.git \
    && cd MonoGS && git checkout "$MONOGS_COMMIT" && git submodule update --init --recursive \
    && git submodule status --recursive

WORKDIR /opt/MonoGS
RUN pip install --no-cache-dir --no-build-isolation submodules/simple-knn submodules/diff-gaussian-rasterization

# Also write the pose of every tracked frame (not only keyframes) next to MonoGS's own trajectory files.
RUN python3 - <<'EOF'
p = "slam.py"
s = open(p).read()
anchor = '        Log("Total FPS", N_frames / (start.elapsed_time(end) * 0.001), tag="Eval")\n'
assert anchor in s
dump = anchor + '''        if self.save_dir is not None:
            import numpy as _np
            with open(os.path.join(self.save_dir, "trj_all_frames.txt"), "w") as _f:
                for _idx in sorted(self.frontend.cameras.keys()):
                    _cam = self.frontend.cameras[_idx]
                    _w2c = _np.eye(4)
                    _w2c[:3, :3] = _cam.R.detach().cpu().numpy()
                    _w2c[:3, 3] = _cam.T.detach().cpu().numpy()
                    _c2w = _np.linalg.inv(_w2c)
                    _f.write(str(_cam.uid) + " " + " ".join("%.9f" % v for v in _c2w[:3].reshape(-1)) + "\\n")
'''
open(p, "w").write(s.replace(anchor, dump))
EOF

# Cache the AlexNet weights used for LPIPS so evaluation needs no network access.
RUN python3 -c "from torchmetrics.image.lpip import LearnedPerceptualImagePatchSimilarity as L; L(net_type='alex', normalize=True)"

COPY run_tum.sh /opt/run_tum.sh
RUN chmod +x /opt/run_tum.sh
CMD ["/opt/run_tum.sh"]
