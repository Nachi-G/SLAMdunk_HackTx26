#!/usr/bin/env bash
set -euo pipefail
car_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v gz >/dev/null; then
  echo 'Install Gazebo Harmonic (Sim 8) on Linux, then run this script there.' >&2
  exit 1
fi
exec gz sim --force-version 8 -r "$car_dir/worlds/house_car.sdf" "$@"
