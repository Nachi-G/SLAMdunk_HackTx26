#!/usr/bin/env bash
set -eo pipefail
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
source px4_ros2_ws/install/setup.bash
{
  date -Is
  echo '=== Actual runtime processes ==='
  pgrep -af '(^|/)(px4|MicroXRCEAgent)( |$)|gz sim|gz-sim|gzserver|gzclient' || true
  echo '=== Controller-environment telemetry ==='
  .dashboard_venv/bin/python diagnostics/probe_demo_telemetry.py
  echo '=== System Gazebo discovery ==='
  source scripts/gazebo_system_env.sh
  timeout 5 /usr/bin/gz topic -l || true
} 2>&1 | tee diagnostics/frontend/runtime_probe.log
