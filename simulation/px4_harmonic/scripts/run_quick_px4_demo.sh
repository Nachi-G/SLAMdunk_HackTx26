#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/project_paths.sh"
# Default: preview. Use --fly only after explicit approval for this flight test.
set -eo pipefail
cd "${task_project_root}"
exec 9>/tmp/hacktx_quick_px4_demo.lock
if ! flock -n 9; then
  echo "Another demo instance is running; close that instance before retrying." >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "${PX4_MSGS_WS}/install/setup.bash"
mkdir -p diagnostics/quick_px4_demo
export ROS_LOG_DIR=${task_project_root}/diagnostics/quick_px4_demo/ros_logs
# Reuse the existing DDS agent; start it only if this endpoint is absent.
if ! pgrep -f '^([^ ]*/)?MicroXRCEAgent udp4 -p 8888([ ]|$)' >/dev/null; then
  echo 'Starting DDS agent on UDP port 8888...'
  nohup /usr/local/bin/MicroXRCEAgent udp4 -p 8888 \
    > diagnostics/quick_px4_demo/dds_agent.log 2>&1 < /dev/null 9>&- &
  task_agent_pid=$!
  sleep 0.5
  if ! kill -0 "$task_agent_pid" 2>/dev/null; then
    echo 'DDS agent exited; inspect diagnostics/quick_px4_demo/dds_agent.log' >&2
    exit 1
  fi
  echo "DDS agent PID: $task_agent_pid (left running for PX4/ROS)"
else
  echo 'Using the existing DDS agent on UDP port 8888.'
fi
if [ ! -x quick_px4_demo/obstacle_rpc ] || [ quick_px4_demo/obstacle_rpc.cpp -nt quick_px4_demo/obstacle_rpc ]; then
  bash scripts/build_obstacle_rpc.sh
fi
task_python=python3
for task_arg in "$@"; do
  if [ "$task_arg" = '--manual-control' ]; then
    if [ ! -x .dashboard_venv/bin/python ]; then
      echo 'Run bash ~/hacktx/scripts/start_live_dashboard.sh first to prepare the dashboard environment.' >&2
      exit 1
    fi
    task_python=.dashboard_venv/bin/python
  fi
done
"$task_python" quick_px4_demo/controller.py "$@" 2>&1 | tee -i diagnostics/quick_px4_demo/console.log
