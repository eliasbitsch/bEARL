# FAST-LIO2 exactly as the M3DGR authors adapted it (baseline_systems/Fast_LIO2_M3DGR, MID-360 config), ROS 1 Noetic.
# Build:  docker build -t bearl/fast_lio2 docker/fast_lio2
# Run:    see run.sh (the original ROS 1 bag is mounted read-only).
FROM ros:noetic-ros-base

ARG M3DGR_COMMIT=5fc892c84defef42feb2418f7d38897bef40f490
ARG LIVOX_SDK_TAG=v2.3.0

RUN apt-get update && apt-get install -y --no-install-recommends \
        git cmake build-essential libeigen3-dev libpcl-dev \
        ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-tf ros-noetic-eigen-conversions \
        ros-noetic-rosbag ros-noetic-rostopic ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs \
        ros-noetic-topic-tools ros-noetic-nodelet \
    && rm -rf /var/lib/apt/lists/*

# livox_ros_driver (needed for the CustomMsg in the bags) links against the Livox SDK
RUN git clone --depth 1 --branch ${LIVOX_SDK_TAG} https://github.com/Livox-SDK/Livox-SDK /tmp/Livox-SDK \
    && cmake -S /tmp/Livox-SDK -B /tmp/Livox-SDK/build -DCMAKE_BUILD_TYPE=Release \
    && make -C /tmp/Livox-SDK/build -j"$(nproc)" install && rm -rf /tmp/Livox-SDK

RUN git clone --filter=blob:none --no-checkout https://github.com/sjtuyinjie/M3DGR /opt/M3DGR \
    && cd /opt/M3DGR && git sparse-checkout set --no-cone baseline_systems/Fast_LIO2_M3DGR \
    && git checkout ${M3DGR_COMMIT} && echo ${M3DGR_COMMIT} > /opt/M3DGR_COMMIT

WORKDIR /opt/M3DGR/baseline_systems/Fast_LIO2_M3DGR
RUN . /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)"

COPY run.sh /opt/run.sh
RUN chmod +x /opt/run.sh
ENTRYPOINT ["/opt/run.sh"]
