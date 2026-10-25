#!/usr/bin/env bash
# Reads existing SANDO/PX4 telemetry; creates no publishers.
set -e
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
source decomp_ws/install/setup.bash
source sando_harmonic_ws/install/setup.bash
source px4_ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/harneksab/hacktx/diagnostics/live_bridge_ros_logs
python3 sando_to_px4_dry_run.py --seconds 15
