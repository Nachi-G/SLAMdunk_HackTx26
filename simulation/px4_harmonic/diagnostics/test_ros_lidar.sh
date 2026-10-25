#!/usr/bin/env bash
# Start a separate sensor-only test; never connects to PX4 or planner inputs.
set -e
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix ros_gz_bridge >/dev/null
python3 diagnostics/run_ros_lidar_test.py
