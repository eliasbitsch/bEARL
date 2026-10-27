#!/usr/bin/env bash
# usage: play.sh <bag folder under /data> [extra ros2 bag play args]
set -eo pipefail
source /opt/ros/jazzy/setup.bash
BAG="/data/${1:?bag folder under /data, e.g. Dynamic01_ros2}"
shift
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=${PORT:-8765} &
# Livox CustomMsg is not installed in a stock ROS; its PointCloud2 twins (<topic>_points) carry the same scans
exec ros2 bag play "$BAG" --loop --clock --exclude-regex '/livox/(mid360|avia)/lidar$' "$@"
