#!/usr/bin/env bash
# Records one failed solver model from an isolated SANDO instance only.
set -e
cd /home/harneksab/hacktx
source /opt/ros/jazzy/setup.bash
source decomp_ws/install/setup.bash
source sando_harmonic_ws/install/setup.bash
export GUROBI_HOME=/home/harneksab/hacktx/gurobi1103/linux64
export LD_LIBRARY_PATH="$GUROBI_HOME/lib:${LD_LIBRARY_PATH:-}"
python3 diagnostics/run_ros_lidar_test.py --planner --capture-solver
# Disable dual reductions to distinguish infeasibility from unboundedness.
# .ilp asks Gurobi to identify and save conflicting constraints.
printf 'Analyzing captured constraints (up to 35 seconds)...\n'
timeout 35 "$GUROBI_HOME/bin/gurobi_cl" DualReductions=0 TimeLimit=25 Threads=1 \
  ResultFile=diagnostics/solver_capture/conflict.ilp \
  diagnostics/solver_capture/failed_model.lp \
  > diagnostics/solver_capture/conflict_analysis.log 2>&1 || status=$?
printf 'Diagnostic files saved under diagnostics/solver_capture/\n'
