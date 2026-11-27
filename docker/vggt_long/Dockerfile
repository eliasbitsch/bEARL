# VGGT-Long (Deng et al., ICRA 2026), headless build for benchmarking.
#
# Build (from the repo root):
#   docker build -t bearl/vggt_long docker/vggt_long
# Run: see run.sh (weights, image folder and output folder are mounted at runtime).
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

ARG VGGT_LONG_COMMIT=c160869d1d99c96bb227f414afb3bc68c29c9a76

# 8.6 = RTX 30xx (laptop tests), 9.0 = H100/H200 (cluster)
ENV DEBIAN_FRONTEND=noninteractive \
    TORCH_CUDA_ARCH_LIST="8.6;9.0" \
    MAX_JOBS=8 \
    PATH=/opt/conda/bin:$PATH \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        git wget ca-certificates build-essential ninja-build libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN wget -q https://github.com/conda-forge/miniforge/releases/download/24.11.3-0/Miniforge3-24.11.3-0-Linux-x86_64.sh -O /tmp/mf.sh \
    && bash /tmp/mf.sh -b -p /opt/conda && rm /tmp/mf.sh \
    && conda install -y python=3.10.18 && conda clean -afy

# Versions from the upstream README (torch 2.5.1); CUDA 12.4 wheels instead of 11.8 for the H200.
RUN pip install --no-cache-dir torch==2.5.1 torchvision==0.20.1 \
        --index-url https://download.pytorch.org/whl/cu124 \
    && pip install --no-cache-dir xformers==0.0.28.post3 --index-url https://download.pytorch.org/whl/cu124

WORKDIR /opt
RUN git clone https://github.com/DengKaiCQ/VGGT-Long.git && cd VGGT-Long && git checkout ${VGGT_LONG_COMMIT}

WORKDIR /opt/VGGT-Long
# faiss-gpu has no wheel for current Python/CUDA; the upstream README suggests faiss-gpu-cu12.
# xformers and torch are already pinned above. Imported by the code but missing from requirements.txt:
# opencv, pypose, trimesh, matplotlib (opencv < 4.11 keeps numpy 1.x, which upstream pins).
RUN sed -i -e 's/^faiss-gpu$/faiss-gpu-cu12/' -e '/^xformers/d' requirements.txt \
    && pip install --no-cache-dir -r requirements.txt "safetensors==0.5.3" pyyaml \
        "opencv-python-headless==4.10.0.84" "pypose==0.9.5" trimesh "matplotlib<3.10" "numpy==1.26.1" \
    && python setup.py install \
    && python -c "import torch, sim3solve, faiss, numpy; print('sim3solve + faiss ok, torch', torch.__version__, 'numpy', numpy.__version__)" \
    && python -c "import vggt_long, base_models.base_model, loop_utils.sim3loop, LoopModels.LoopModel; print('VGGT-Long imports ok')"

COPY run.sh /opt/run.sh
RUN chmod +x /opt/run.sh

ENTRYPOINT ["/opt/run.sh"]
