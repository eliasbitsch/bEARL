#!/bin/bash
# Runs inside bearl/glim_demo: GLIM (live node) + static TF map->world + RViz, then plays the demo bag.
# Mounted: /glim/config (repo config, read-only), /demo (this folder), /data (sequence folder).
set -e
source /opt/ros/jazzy/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash

# Demo config = pipeline config + RViz publisher; IMU and LiDAR share "livox_frame" in M3DGR,
# so give the IMU its own TF name (TF cannot link a frame to itself). No effect on the estimate.
cp -r /glim/config /tmp/config
sed -i 's|"libmemory_monitor.so"|"libmemory_monitor.so", "librviz_viewer.so"|' /tmp/config/config_ros.json
sed -i 's|"imu_frame_id": ""|"imu_frame_id": "mid360_imu"|' /tmp/config/config_ros.json

ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/tmp/config -p use_sim_time:=true &
GLIM=$!
ros2 run tf2_ros static_transform_publisher $(awk '{printf "--x %s --y %s --z %s --qx %s --qy %s --qz %s --qw %s", $1,$2,$3,$4,$5,$6,$7}' /data/glim_demo_tf.txt) \
  --frame-id map --child-frame-id world --ros-args -p use_sim_time:=true &
TF=$!
python3 /demo/accumulate_map.py --ros-args -p use_sim_time:=true &
ACC=$!
rviz2 -d /demo/glim_demo.rviz --ros-args -p use_sim_time:=true &
RVIZ=$!
sleep 8
ros2 bag play /data/glim_demo --clock 200 --rate "${RATE:-1.0}"
echo "Playback finished. Close the RViz window to quit."
wait $RVIZ
kill $GLIM $TF $ACC 2>/dev/null || true
