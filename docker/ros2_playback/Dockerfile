# Play a prepared ROS 2 sequence (python -m pipeline.prepare_data --ros2) and serve it to Foxglove / Lichtblick.
#
#   docker build -t bearl/ros2_playback docker/ros2_playback
#   docker run --rm --network host -v $PWD/data/M3DGR:/data:ro bearl/ros2_playback Dynamic01_ros2
#   -> in Lichtblick / Foxglove: open connection, Foxglove WebSocket, ws://localhost:8765
#   (port taken by another bridge? add -e PORT=8766)
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-foxglove-bridge ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

COPY play.sh /play.sh
RUN chmod +x /play.sh
ENTRYPOINT ["/play.sh"]
