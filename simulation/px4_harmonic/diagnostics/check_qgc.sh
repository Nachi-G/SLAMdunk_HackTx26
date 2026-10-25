#!/usr/bin/env bash
# Read-only PX4 MAVLink / QGC diagnostics. No mode, arm, parameter or flight commands.
set -u
cd /home/harneksab/hacktx || exit 1
mkdir -p diagnostics/qgc
{
  printf '### PX4 MAVLink status\n'
  timeout 8 /home/harneksab/PX4-Autopilot/build/px4_sitl_default/bin/px4-mavlink status 2>&1 || true
  printf '\n### PX4 commander status\n'
  timeout 5 /home/harneksab/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander status 2>&1 || true
  printf '\n### Linux QGC, PX4 and DDS processes\n'
  ps -eo pid,comm,args | awk '$2 ~ /QGround|px4|MicroXRCE/ {print}'
  printf '\n### MAVLink UDP sockets\n'
  ss -uapn 2>&1 | grep -E ':(14550|18570|18571|14540|14580)([[:space:]]|$)' || true
  printf '\n### WSL network routes\n'
  ip -4 route 2>&1 || true
  printf '\n### Windows QGC process\n'
  if [ -x /mnt/c/Windows/System32/tasklist.exe ]; then
    timeout 5 /mnt/c/Windows/System32/tasklist.exe /FI 'IMAGENAME eq QGroundControl.exe' /NH 2>&1 || true
  fi
} > diagnostics/qgc/status.txt
printf 'Read-only QGC diagnostics saved to ~/hacktx/diagnostics/qgc/status.txt\n'
