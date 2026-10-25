#!/usr/bin/env bash
# Read-only verification of the newly attached sensor and vehicle status.
set -u
cd /home/harneksab/hacktx || exit 1
source scripts/gazebo_system_env.sh
mkdir -p diagnostics/x500_lidar
for entry in 'scan /sando/lidar/scan' 'points /sando/lidar/scan/points'; do
  read -r label topic <<< "$entry"
  timeout 5 /usr/bin/gz topic -i -t "$topic" > "diagnostics/x500_lidar/${label}_info.txt" 2>&1
done
timeout 12 /usr/bin/gz topic -e --json-output -n 3 -t /sando/lidar/scan > diagnostics/x500_lidar/scans.jsonl 2> diagnostics/x500_lidar/scan_error.txt
timeout 8 /usr/bin/gz topic -e --json-output -n 3 -t /world/default/pose/info > diagnostics/x500_lidar/poses.jsonl 2> diagnostics/x500_lidar/pose_error.txt
set +u
source /opt/ros/jazzy/setup.bash
source decomp_ws/install/setup.bash
source px4_ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/harneksab/hacktx/diagnostics/x500_lidar/ros_logs
python3 sando_to_px4_dry_run.py --seconds 5 --output diagnostics/x500_lidar/status.json
printf 'Captured sensor and vehicle diagnostics under diagnostics/x500_lidar/\n'
