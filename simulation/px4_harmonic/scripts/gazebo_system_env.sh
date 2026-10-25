# Source inside the launch scripts only; preserve the caller's terminal settings.
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/project_paths.sh"
export GZ_CONFIG_PATH=/usr/share/gz
export PATH="/usr/bin:/bin:$PATH"
# System Gazebo loads its system libraries; ROS processes retain their own setup.
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
  task_library_path=''
  IFS=: read -r -a task_library_entries <<< "$LD_LIBRARY_PATH"
  for task_library_entry in "${task_library_entries[@]}"; do
    if [[ -n "$task_library_entry" && "$task_library_entry" != /opt/ros/* ]]; then
      task_library_path="${task_library_path:+$task_library_path:}$task_library_entry"
    fi
  done
  export LD_LIBRARY_PATH="$task_library_path"
fi
export GZ_SIM_RESOURCE_PATH="${task_project_root}/gazebo_models:${PX4_SOURCE_DIR}/Tools/simulation/gz/models:${HOME}/.simulation-gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
