#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/project_paths.sh"
# Standby controller: remains grounded until Start is clicked in the dashboard.
set -eo pipefail
exec bash "${task_project_root}/scripts/run_quick_px4_demo.sh" --fly --challenge --manual-control --wait-for-start "$@"
