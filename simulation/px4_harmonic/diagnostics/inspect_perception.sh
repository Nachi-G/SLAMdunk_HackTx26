#!/usr/bin/env bash
# Read-only diagnostics. Does not launch/stop simulation or publish ROS messages.
set -u
cd /home/harneksab/hacktx || exit 1
section() { printf '\n### %s\n' "$1"; }
section 'Visible robotics processes'
ps -eo pid,comm,args | awk '$2 ~ /^(px4|gz|gz-sim.*|ruby|MicroXRCEAgent|sando|python3|parameter_bridg)$/ {print}'
section 'Gazebo topic list'
timeout 12 gz topic -l
section 'Previously observed LiDAR endpoints'
for topic in /world/default/model/x500_0/link/lidar_sensor_link/sensor/lidascan /world/default/model/x500_0/link/link/sensor/lidar_2d_v2/scan; do
  printf '\n%s\n' "$topic"
  timeout 5 gz topic -i -t "$topic"
done
section 'ROS environment'
for setup in /opt/ros/jazzy/setup.bash /home/harneksab/hacktx/decomp_ws/install/setup.bash /home/harneksab/hacktx/sando_harmonic_ws/install/setup.bash /home/harneksab/hacktx/px4_ros2_ws/install/setup.bash; do
  if [ -f "$setup" ]; then
    set +u
    source "$setup"
    set -u
  fi
done
section 'ROS topics'
timeout 12 ros2 topic list --no-daemon
section 'Point cloud connections'
timeout 12 ros2 topic info /NX01/sensor_point_cloud --verbose
section 'Bridge installation'
ros2 pkg prefix ros_gz_bridge
section 'Recent Gazebo sensor/rendering errors'
if [ -d "$HOME/.gz" ]; then
  rg --glob '*.log' --max-count 8 -i 'error|failed|ogre|lidar|render' "$HOME/.gz" 2>/dev/null | tail -80
fi
section 'Complete'
