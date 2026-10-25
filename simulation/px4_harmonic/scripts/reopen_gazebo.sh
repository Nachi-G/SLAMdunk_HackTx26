#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/project_paths.sh"
# Recover the GUI without restarting a running simulator or issuing flight commands.
set -euo pipefail
cd "${task_project_root}"
source scripts/gazebo_system_env.sh
service_info=$(timeout 5 /usr/bin/gz service -i -s /world/default/scene/info 2>&1) || true
if [[ "$service_info" == *'Service providers'* ]]; then
  echo 'Gazebo server is still running. Reopening its GUI; leave PX4 running.'
  export LIBGL_ALWAYS_SOFTWARE=1
  exec /usr/bin/gz sim -g
fi
if pgrep -x px4 >/dev/null; then
  cat <<'TEXT'
Gazebo's server appears stopped, but PX4 is still running.
If the drone was landed and disarmed, press Ctrl+C in the PX4 terminal.
Then run this recovery script again.
After the forest appears, restart PX4 in its terminal with:
  bash ~/hacktx/scripts/start_px4_lidar.sh
No PX4 process or flight mode has been changed by this script.
TEXT
  exit 1
fi
cat <<'TEXT'
Starting the LiDAR forest world.
After the forest and drone appear, start PX4 in a separate terminal with:
  bash ~/hacktx/scripts/start_px4_lidar.sh
TEXT
exec bash "${task_project_root}/scripts/start_lidar_forest.sh"
