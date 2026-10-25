#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/project_paths.sh"
# Attach the existing X500 airframe to the LiDAR model already in the forest.
# Does not send arm, takeoff, mode, or setpoint commands.
set -euo pipefail
source "${task_project_root}/scripts/gazebo_system_env.sh"
if pgrep -x px4 >/dev/null; then
  echo 'An existing PX4 instance is running; refusing to start a second one.' >&2
  exit 1
fi
export PX4_GZ_NO_FOLLOW=1
export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD=default
export PX4_GZ_MODEL_NAME=x500_0
cd "${PX4_SOURCE_DIR}"
exec make px4_sitl gz_x500
