#!/usr/bin/env bash
# Telemetry only; no landing/disarm/mode commands.
set -e
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
source px4_ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/harneksab/hacktx/diagnostics/ground_check_ros_logs
python3 diagnostics/check_grounded.py
