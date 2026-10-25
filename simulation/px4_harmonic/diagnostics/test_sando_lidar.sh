#!/usr/bin/env bash
# Sensor-only forest + isolated planner; never starts or connects to PX4.
set -e
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
source decomp_ws/install/setup.bash
source sando_harmonic_ws/install/setup.bash
export GUROBI_HOME=/home/harneksab/hacktx/gurobi1103/linux64
export LD_LIBRARY_PATH="$GUROBI_HOME/lib:${LD_LIBRARY_PATH:-}"
python3 diagnostics/run_ros_lidar_test.py --planner "$@"
