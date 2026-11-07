#!/usr/bin/env bash
# Replay one M3DGR bag through FAST-LIO2 (MID-360 config of the M3DGR authors).
#   /data/<seq>.bag   original ROS 1 bag (read-only)       /out   odom.csv, scans.pcd, log.txt
# Env: SEQ (bag name without .bag), RATE (rosbag play rate, default 1.0), HOST_UID / HOST_GID
set -eo pipefail
source /opt/M3DGR/baseline_systems/Fast_LIO2_M3DGR/devel/setup.bash
WS=/opt/M3DGR/baseline_systems/Fast_LIO2_M3DGR
mkdir -p "$WS/src/fast_lio2/PCD"
echo "M3DGR commit: $(cat /opt/M3DGR_COMMIT)" | tee /out/log.txt

roscore > /dev/null 2>&1 &
until rostopic list > /dev/null 2>&1; do sleep 0.5; done
roslaunch fast_lio mapping_mid360.launch rviz:=false >> /out/log.txt 2>&1 &
LAUNCH=$!
rostopic echo -p /Odometry > /out/odom.csv &
ECHO=$!
sleep 3
rosbag play --quiet -r "${RATE:-1.0}" "/data/${SEQ:?SEQ not set}.bag" --topics /livox/mid360/lidar /livox/mid360/imu
sleep 3                                        # let the last scans be processed

kill "$ECHO" || true
kill -INT "$LAUNCH"                            # FAST-LIO writes PCD/scans.pcd on shutdown
wait "$LAUNCH" || true
cp "$WS/src/fast_lio2/PCD/scans.pcd" /out/scans.pcd 2>/dev/null || echo "no scans.pcd written" | tee -a /out/log.txt
echo "poses: $(($(wc -l < /out/odom.csv) - 1))" | tee -a /out/log.txt

if [ -n "${HOST_UID:-}" ]; then
    chown -R "${HOST_UID}:${HOST_GID:-$HOST_UID}" /out
fi
