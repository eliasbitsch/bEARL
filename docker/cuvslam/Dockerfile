# cuVSLAM / PyCuVSLAM (NVIDIA Community License, runs on NVIDIA GPUs only)
# Upstream: https://github.com/nvidia-isaac/cuVSLAM, prebuilt wheel from release v17.0.0
# Wheel: CUDA 12, CPython 3.10, manylinux_2_35 (glibc 2.35 = Ubuntu 22.04)
#
# The wheel is downloaded on the host first (GitHub release downloads inside the
# build kept failing with connection resets) and passed in as a named build context:
#   W=cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_x86_64.whl
#   curl -L -C - -o wheels/$W https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.0/${W/+/%2B}
#   docker build --build-context wheels=wheels -t bearl/cuvslam:17.0.0 docker/cuvslam
FROM nvidia/cuda:12.6.3-runtime-ubuntu22.04

ARG CUVSLAM_WHEEL=cuvslam-17.0.0+cu12-cp310-cp310-manylinux_2_35_x86_64.whl
ARG CUVSLAM_SHA256=0ae3113e43902e0c15423992450510aef241f84207718ed77a1d2d39f28744c1

ENV DEBIAN_FRONTEND=noninteractive PIP_NO_CACHE_DIR=1
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip curl ca-certificates libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=wheels ${CUVSLAM_WHEEL} /tmp/
RUN echo "${CUVSLAM_SHA256}  /tmp/${CUVSLAM_WHEEL}" | sha256sum -c - \
    && python3 -m pip install --retries 10 --timeout 60 /tmp/${CUVSLAM_WHEEL} \
        numpy==2.2.6 pillow==12.3.0 opencv-python-headless~=4.11.0 \
    && rm /tmp/${CUVSLAM_WHEEL}

COPY run_tum.py /opt/cuvslam/run_tum.py
WORKDIR /opt/cuvslam
ENTRYPOINT ["python3", "/opt/cuvslam/run_tum.py"]
