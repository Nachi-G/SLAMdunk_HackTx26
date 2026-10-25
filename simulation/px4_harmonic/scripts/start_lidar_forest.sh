#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/project_paths.sh"
# Run only after the previous simulation is stopped while landed/disarmed.
set -euo pipefail
cd "${task_project_root}"
source scripts/gazebo_system_env.sh
if pgrep -x px4 >/dev/null; then
  echo 'An existing PX4 instance is running. Stop it only after confirmed landing/disarming, then retry.' >&2
  exit 1
fi
services=$(timeout 5 /usr/bin/gz service -i -s /world/default/scene/info 2>&1) || true
if [[ "$services" == *'Service providers'* ]]; then
  echo 'A default-world Gazebo server is already running. Stop the old landed/disarmed simulation first.' >&2
  exit 1
fi
export LIBGL_ALWAYS_SOFTWARE=1
exec /usr/bin/gz sim -r -v 3 "${task_project_root}/gazebo_worlds/hacktx_forest_x500_sando.sdf"
