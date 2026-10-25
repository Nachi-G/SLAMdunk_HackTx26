#!/usr/bin/env bash
# Read-only: planner parameters, topic connections, synchronized receipt observations.
set -e
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
source decomp_ws/install/setup.bash
source sando_harmonic_ws/install/setup.bash
source px4_ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/harneksab/hacktx/diagnostics/live_bridge_ros_logs
mkdir -p diagnostics/alignment
# Save the prior evidence before writing a new report.
if [ -f diagnostics/live_bridge_preview.json ]; then
  cp diagnostics/live_bridge_preview.json diagnostics/alignment/previous_preview.json
fi
timeout 8 ros2 param dump /NX01/sando_node > diagnostics/alignment/planner_parameters.yaml 2> diagnostics/alignment/parameter_error.txt || true
timeout 5 ros2 topic info /NX01/state --verbose > diagnostics/alignment/state_connections.txt 2>&1 || true
timeout 5 ros2 topic info /trajs --verbose > diagnostics/alignment/trajectory_connections.txt 2>&1 || true
python3 sando_to_px4_dry_run.py --seconds 15 --output diagnostics/alignment/observations.json
python3 diagnostics/analyze_alignment.py
