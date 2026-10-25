std_msgs/Header header
bool result                          # replanning success/failure
float64 successful_factor            # time allocation factor that succeeded
float64 total_replanning_ms          # total replanning wall-clock time
float64 global_planning_ms           # global path search
float64 hgp_static_jps_ms            # static JPS sub-step
float64 hgp_check_path_ms            # path validity check
float64 hgp_dynamic_astar_ms         # dynamic A* sub-step
float64 hgp_recover_path_ms          # path recovery sub-step
float64 cvx_decomp_ms                # convex decomposition
float64 local_traj_ms                # Gurobi local trajectory optimization
float64 safe_paths_ms                # safe path computation
float64 safety_check_ms              # safety check
float64 yaw_sequence_ms              # yaw sequence generation
float64 yaw_fitting_ms               # yaw fitting
