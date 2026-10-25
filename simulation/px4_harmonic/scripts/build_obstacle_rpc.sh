#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
c++ -std=c++17 -O2 quick_px4_demo/obstacle_rpc.cpp -o quick_px4_demo/obstacle_rpc $(pkg-config --cflags --libs gz-transport13 gz-msgs10)
