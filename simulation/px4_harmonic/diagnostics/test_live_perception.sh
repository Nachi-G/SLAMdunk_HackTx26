#!/usr/bin/env bash
# Mounted LiDAR -> separate PX4-ENU preview only. No flight or planner commands.
set -eo pipefail
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
source decomp_ws/install/setup.bash
source px4_ros2_ws/install/setup.bash
mkdir -p diagnostics/live_perception
export ROS_LOG_DIR=/home/harneksab/hacktx/diagnostics/live_perception/ros_logs
/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge --ros-args \
  -r __node:=x500_live_points_preview_bridge \
  -p config_file:=/home/harneksab/hacktx/config/sando_lidar_preview_bridge.yaml \
  > diagnostics/live_perception/bridge.log 2>&1 &
bridge_pid=$!
trap 'kill -INT "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true' EXIT
python3 x500_live_perception_preview.py --seconds 20 2>&1 | tee diagnostics/live_perception/console.log
