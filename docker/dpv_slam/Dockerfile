# DPV-SLAM (Deep Patch Visual SLAM, ECCV 2024) and DPVO, headless (no Pangolin viewer, no DBoW2 backend).
# Upstream: https://github.com/princeton-vl/DPVO  (tested upstream with python 3.10, torch 2.3.1, CUDA 12.1)
#
# Build:  docker build -t bearl/dpv_slam docker/dpv_slam
# Run:    see docker/dpv_slam/run_tum.sh
FROM nvidia/cuda:12.1.1-devel-ubuntu22.04

ARG DPVO_COMMIT=0ac95b656d1fda91c271d2a106460d19ad966fc7
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PIP_DEFAULT_TIMEOUT=120 \
    PIP_RETRIES=20 \
    PIP_RESUME_RETRIES=200 \
    TORCH_CUDA_ARCH_LIST="8.6;9.0" \
    MAX_JOBS=8 \
    NUMBA_CACHE_DIR=/tmp/numba_cache

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3.10 python3.10-dev python3-pip git wget unzip ca-certificates libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/* \
    && ln -sf /usr/bin/python3.10 /usr/bin/python

# Versions from upstream environment.yml (pytorch 2.3.1, torchvision 0.18, pytorch-scatter 2.1.2, numpy 1.26.4).
RUN printf "torch==2.3.1\ntorchvision==0.18.1\nnumpy==1.26.4\n" > /opt/constraints.txt \
    && python -m pip install --upgrade pip setuptools wheel \
    && python -m pip install torch==2.3.1 torchvision==0.18.1 --index-url https://download.pytorch.org/whl/cu121 \
    && python -m pip install -c /opt/constraints.txt torch-scatter==2.1.2 -f https://data.pyg.org/whl/torch-2.3.0+cu121.html \
    && python -m pip install -c /opt/constraints.txt numpy==1.26.4 numba tqdm einops pypose kornia plyfile evo \
        opencv-python-headless yacs scipy

RUN git clone https://github.com/princeton-vl/DPVO.git /opt/DPVO \
    && cd /opt/DPVO && git checkout ${DPVO_COMMIT} \
    && wget -q https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip \
    && unzip -q eigen-3.4.0.zip -d thirdparty && rm eigen-3.4.0.zip \
    && python -m pip install -c /opt/constraints.txt --no-build-isolation . \
    && python -c "import torch, cuda_corr, cuda_ba, lietorch_backends, torch_scatter; print(torch.__version__)"

WORKDIR /opt/DPVO
COPY run_tum.sh /opt/run_tum.sh
RUN chmod +x /opt/run_tum.sh
