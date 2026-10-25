#!/usr/bin/env bash
# Usage: bash simulation/house/run_house.sh [--static] [Gazebo arguments...]
set -euo pipefail
house_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
world="$house_dir/worlds/house.sdf"
if [[ "${1:-}" == "--static" ]]; then
  world="$house_dir/worlds/house_static.sdf"
  shift
fi
if ! command -v gz >/dev/null 2>&1; then
  echo 'Gazebo Harmonic is required (gz-sim 8; Ubuntu package: gz-harmonic).' >&2
  exit 127
fi
exec gz sim --force-version 8 -r "$world" "$@"
