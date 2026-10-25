#!/usr/bin/env bash
# Source from runtime launchers; no processes or flight commands are started here.
task_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PX4_SOURCE_DIR="${PX4_SOURCE_DIR:-$HOME/PX4-Autopilot}"
export HACKTX_WORKSPACE="${HACKTX_WORKSPACE:-$HOME/hacktx}"
export PX4_MSGS_WS="${PX4_MSGS_WS:-$HACKTX_WORKSPACE/px4_ros2_ws}"
