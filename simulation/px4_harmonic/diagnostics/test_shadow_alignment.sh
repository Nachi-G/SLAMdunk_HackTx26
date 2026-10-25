#!/usr/bin/env bash
# Separate planner and read-only bridge observer; no PX4 command publishers.
set -e
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
source decomp_ws/install/setup.bash
source sando_harmonic_ws/install/setup.bash
source px4_ros2_ws/install/setup.bash
export GUROBI_HOME=/home/harneksab/hacktx/gurobi1103/linux64
export LD_LIBRARY_PATH="$GUROBI_HOME/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR=/home/harneksab/hacktx/diagnostics/shadow_alignment/ros_logs
python3 diagnostics/run_shadow_alignment.py
