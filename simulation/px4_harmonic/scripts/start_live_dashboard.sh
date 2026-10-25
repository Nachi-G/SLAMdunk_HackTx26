#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/project_paths.sh"
set -eo pipefail
cd "${task_project_root}"
# Isolated environment; system packages retain Gazebo transport Python bindings.
if [ ! -x .dashboard_venv/bin/python ]; then
  /usr/bin/python3 -m venv --system-site-packages .dashboard_venv
fi
if ! .dashboard_venv/bin/python -c 'import websockets; assert websockets.__version__ == "13.1"' 2>/dev/null; then
  .dashboard_venv/bin/python -m pip install 'websockets==13.1'
fi
source scripts/gazebo_system_env.sh
mkdir -p diagnostics/frontend
.dashboard_venv/bin/python -u frontend_adapter/gazebo_live.py 2>&1 | tee diagnostics/frontend/live_dashboard.log
