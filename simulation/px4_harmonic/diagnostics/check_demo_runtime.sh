#!/usr/bin/env bash
# Read-only checks. Does not arm, change mode, move models or stop processes.
set -eo pipefail
cd /home/harneksab/hacktx
mkdir -p diagnostics/frontend
source /opt/ros/jazzy/setup.bash
source px4_ros2_ws/install/setup.bash
{
  date -Is
  echo '=== Relevant processes ==='
  ps -eo pid,comm,args | grep -E '(^|/)(px4 |MicroXRCEAgent |gz sim |gz-sim|gzserver)|quick_px4_demo/controller|gazebo_live.py' | grep -v grep || true
  echo '=== Network/domain settings in this terminal ==='
  env | grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|ROS_AUTOMATIC_DISCOVERY_RANGE|RMW_IMPLEMENTATION|GZ_PARTITION|GZ_IP|IGN_PARTITION)=' || true
  echo '=== Gazebo pose publishers ==='
  timeout 6 /usr/bin/gz topic -i -t /world/default/pose/info || true
  echo '=== ROS topics (fresh discovery) ==='
  timeout 12 ros2 topic list --no-daemon -t || true
  echo '=== DDS endpoint ==='
  ss -ulnp | grep ':8888' || true
} 2>&1 | tee diagnostics/frontend/runtime_check.log
