# MASt3R-SLAM (Murai, Dexheimer, Davison, CVPR 2025), headless build for benchmarking.
#
# Build:
#   docker build -t bearl/mast3r_slam docker/mast3r_slam
# Run: see run.sh (image folder, checkpoints and output folder are mounted at runtime); the pipeline runner
# mounts the repo run.sh over the baked-in copy.
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

ARG MAST3R_SLAM_COMMIT=e6f4e3d474fad0e11f561482012be864ba8c3f17
ARG MAST3R_SLAM_WINDOWS_COMMIT=6717231a2daf55d501a5824bbec43314d4fb77d9

ENV DEBIAN_FRONTEND=noninteractive \
    TORCH_CUDA_ARCH_LIST="8.6;9.0" \
    MAX_JOBS=8 \
    PATH=/opt/conda/bin:$PATH \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        git wget ca-certificates build-essential ninja-build \
        libgl1 libegl1 libglib2.0-0 libsm6 libxext6 libxrender1 libx11-6 libglfw3 \
    && rm -rf /var/lib/apt/lists/*

RUN wget -q https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -O /tmp/mf.sh \
    && bash /tmp/mf.sh -b -p /opt/conda && rm /tmp/mf.sh \
    && conda install -y python=3.11 && conda clean -afy

RUN pip install --no-cache-dir torch==2.5.1 torchvision==0.20.1 \
        --index-url https://download.pytorch.org/whl/cu124 \
    && pip install --no-cache-dir "setuptools==70.0.0" wheel "numpy==1.26.4"

WORKDIR /opt
RUN git clone https://github.com/rmurai0610/MASt3R-SLAM.git \
    && cd MASt3R-SLAM \
    && git checkout ${MAST3R_SLAM_COMMIT} \
    && git submodule update --init --recursive \
    && git fetch origin windows \
    && git show ${MAST3R_SLAM_WINDOWS_COMMIT}:main.py > main_windows.py

WORKDIR /opt/MASt3R-SLAM
# setup.py only builds the CUDA backend when a GPU is visible; there is none during docker build.
RUN sed -i 's/^has_cuda = torch.cuda.is_available()/has_cuda = True/' setup.py \
    && pip install --no-cache-dir --no-build-isolation -e thirdparty/mast3r \
    && pip install --no-cache-dir -e thirdparty/in3d \
    && pip install --no-cache-dir --no-build-isolation -e .

# The latest opencv-python / pandas pulled in above need numpy 2, moderngl-window needs numpy < 2: keep 1.26 and pin
# versions built for it. pyrealsense2 (imported by the dataloader) needs libusb. torch must be imported first (libc10).
RUN apt-get update && apt-get install -y --no-install-recommends libusb-1.0-0 && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir "numpy==1.26.4" "opencv-python<4.11" "pandas<2.3" \
    && python -c "import torch, mast3r_slam_backends, lietorch, cv2, pandas; import mast3r_slam.dataloader, mast3r_slam.evaluate; print('imports ok, torch', torch.__version__)"

COPY run.sh /opt/run.sh
RUN chmod +x /opt/run.sh

ENTRYPOINT ["/opt/run.sh"]
