# GLIM image + RViz 2 for the live demo (bag playback -> GLIM -> RViz). Not used for results.
FROM koide3/glim_ros2:jazzy_cuda13.1@sha256:8e00163cd989faf845134500237cfdcc0ed3b14fb3b33ce45b4952ba29b9c8cc
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*
