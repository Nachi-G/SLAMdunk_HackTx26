#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/project_paths.sh"
# Restore only local PX4->QGC telemetry. No arming, mode, movement, or parameter changes.
set -euo pipefail
cd "${task_project_root}"
mkdir -p diagnostics/qgc
px4_bin=${PX4_SOURCE_DIR}/build/px4_sitl_default/bin
if ! timeout 6 "$px4_bin/px4-commander" status > diagnostics/qgc/commander_before.txt 2>&1; then
  cat diagnostics/qgc/commander_before.txt
  echo 'Cannot query the running PX4 instance; leave flight tests stopped.' >&2
  exit 1
fi
if ! grep -Eq '(^|[[:space:]])Disarmed([[:space:]]|$)' diagnostics/qgc/commander_before.txt; then
  echo 'PX4 is not confirmed disarmed. No telemetry reconfiguration performed.' >&2
  exit 1
fi
status_code=0
timeout 6 "$px4_bin/px4-mavlink" status > diagnostics/qgc/mavlink_before.txt 2>&1 || status_code=$?
printf '%s\n' "$status_code" > diagnostics/qgc/mavlink_status_exit_code.txt
if [ "$status_code" -eq 124 ]; then
  echo 'MAVLink status timed out; not starting a duplicate instance. Tell Codex this script stopped here.' >&2
  exit 1
fi
ss -uan > diagnostics/qgc/sockets_before.txt
# A separate explicit loopback link avoids changing any existing link's destination.
if ! grep -Eq ':18571([[:space:]]|$)' diagnostics/qgc/sockets_before.txt; then
  echo 'Starting explicit QGC repair link: UDP18571 -> 127.0.0.1:14550'
  if ! timeout 10 "$px4_bin/px4-mavlink" start -x -u 18571 -o 14550 -t 127.0.0.1 -m normal -r 20000 -f \
      > diagnostics/qgc/start_link.txt 2>&1; then
    cat diagnostics/qgc/start_link.txt
    exit 1
  fi
else
  echo 'Explicit UDP18571 telemetry link already exists; not creating another.'
fi
timeout 6 "$px4_bin/px4-mavlink" boot_complete > diagnostics/qgc/boot_complete.txt 2>&1
sleep 2
timeout 6 "$px4_bin/px4-mavlink" status > diagnostics/qgc/mavlink_after.txt 2>&1 || true
ss -uapn > diagnostics/qgc/sockets_after.txt
cat diagnostics/qgc/mavlink_after.txt
if ! grep -Eq ':14550([[:space:]]|$)' diagnostics/qgc/sockets_after.txt; then
  cat <<'TEXT'
QGroundControl is not listening on UDP14550 yet.
In QGC: Application Settings -> Comm Links -> Add.
Choose UDP, listening port 14550, then save and Connect.
Keep PX4 and Gazebo running; do not start the flight test yet.
TEXT
fi
printf '\nResults saved in ~/hacktx/diagnostics/qgc/\n'

source /opt/ros/jazzy/setup.bash
source "${PX4_MSGS_WS}/install/setup.bash"
export ROS_LOG_DIR=${task_project_root}/diagnostics/qgc/ros_logs
python3 diagnostics/check_gcs_heartbeat.py
