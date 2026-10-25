#!/usr/bin/env bash
# Reads mounted sensor and PX4 telemetry; bridges only simulation data to preview.
set -eo pipefail
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
source decomp_ws/install/setup.bash
source px4_ros2_ws/install/setup.bash
mkdir -p diagnostics/sensor_timing
export ROS_LOG_DIR=/home/harneksab/hacktx/diagnostics/sensor_timing/ros_logs
/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge --ros-args \
  -r __node:=x500_perception_timing_bridge \
  -p config_file:=/home/harneksab/hacktx/config/x500_perception_preview.yaml \
  > diagnostics/sensor_timing/bridge.log 2>&1 &
bridge_pid=$!
trap 'kill -INT "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true' EXIT
python3 diagnostics/capture_sensor_timing.py 2>&1 | tee diagnostics/sensor_timing/capture_console.log
