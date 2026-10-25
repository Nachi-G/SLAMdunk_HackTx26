# Gazebo car — separate from the drone

This folder is a self-contained **four-wheel skid-steer car** for **Gazebo Harmonic / Sim 8**. It turns by driving its left and right wheels at different speeds. It is not an Ackermann steering car.

It includes a furnished static house with the car already inserted, wheel physics, 360° 2D LiDAR, a 16-layer 3D LiDAR, IMU, RGB-D camera, ROS topic bridges, SLAM Toolbox, keyboard driving, a command watchdog, an optional isolated planar SANDO planner, and optional ground frontier exploration. There is no website or CAD viewer. Your original drone code and dashboard are unchanged.

## 1. Put this folder on Ubuntu

Copy the entire `simulation/car` folder to your Linux machine or Ubuntu VM, for example as `~/car`. The model, house and optional planner sources are included; the rest of the repository is not needed to launch this simulation.

The primary target is **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**. Install ROS Jazzy first, then:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-slam-toolbox \
  ros-jazzy-tf2-ros ros-jazzy-sensor-msgs-py \
  python3-colcon-common-extensions python3-rosdep python3-yaml
```

If rosdep has not been initialized on that machine, run `sudo rosdep init` once, then `rosdep update`. Humble with Harmonic requires the non-default compatible `ros_gz` packages; ordinary Humble/Fortress packages are not interchangeable. Gazebo Classic plugins are not used here.

## 2. Build and launch the car + SLAM

From the copied folder:

```bash
cd ~/car
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --base-paths . --packages-select slamdunk_car
source install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch slamdunk_car car.launch.py
```

Gazebo opens the eight-room house, with the car in the middle of the hallway at `(0, 0)`. The house's moving mechanisms are frozen for an initial static baseline. The simulation starts running. SLAM builds its map from sensor scans and wheel odometry; no Gazebo world geometry is passed into SLAM as a map.

In a second terminal, source ROS and `~/car/install/setup.bash`, export `ROS_DOMAIN_ID=20`, then:

```bash
ros2 run slamdunk_car car_teleop
```

Repeat/hold **W** forward, **X** reverse, **A/D** turn, **S** or **Space** stop, **Q** quit. Keyboard requests expire after 0.25 seconds; motion may briefly stop before your OS starts key repeat. The drive guard sends zero if requests, odometry, LiDAR scans, or 3D cloud updates expire. It also stops for an obstacle inside the circular footprint plus braking envelope. View the reason with:

```bash
ros2 topic echo /car/drive_status
```

The guard's stop latch is independent of the keyboard:

```bash
ros2 service call /car/stop std_srvs/srv/SetBool '{data: true}'
# Clear the latch, then issue a NEW command:
ros2 service call /car/stop std_srvs/srv/SetBool '{data: false}'
```

To see just the Gazebo world without ROS or SLAM, use `bash run_gazebo.sh`. The car remains stationary until commanded. `gui:=false` on the ROS launch starts a server without the GUI; GPU LiDAR/RGB-D still need a working rendering backend.

## 3. Optional: build SANDO for the car

`planner/car_sando_planner` contains a separate copy of the required SANDO source, with original BSD notices and a source manifest. It does not build Gazebo Classic plugins, fake_sim, or the drone launch. Its changes are:

- Use the configured planning height instead of the hardcoded 1 m ground height.
- Constrain **all four Bezier control points of every trajectory segment** to that height, making the complete curve planar.
- Configure lower motion limits and a conservative footprint enclosing the full car while it turns.
- Feed SANDO measured SLAM poses and conservative occupied/unknown columns derived from SLAM's grid, augmented by current low-obstacle LiDAR returns. Outside the observed SLAM map remains unknown.
- Track its planar references through bounded forward/turn commands, with angle wrapping, tracking-error rejection, freshness checks, and the same drive guard.

SANDO still requires **a Linux Gurobi installation and an activated license**. Set `GUROBI_HOME` to that installation and configure its license using Gurobi's instructions. The pinned custom message/decomposition dependencies and build adaptations are included under `planner/dependencies`; their source notices are retained.

```bash
cd ~/car
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . planner/car_sando_planner \
  planner/dependencies/dynus_interfaces \
  planner/dependencies/DecompROS2/DecompUtil \
  planner/dependencies/DecompROS2/decomp_ros_msgs \
  --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install \
  --base-paths . planner/car_sando_planner planner/dependencies \
  --packages-up-to slamdunk_car car_sando_planner \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch slamdunk_car car.launch.py mode:=sando
```

Stop any previous car launch first. **Do not run SANDO's normal drone launch, goal_sender, goal_relay, or another mission supervisor for this car.** This launch creates only the dedicated car planner. Keyboard motion is ignored in SANDO mode. The stop service still works.

Send a nearby goal through the hallway, in the SLAM `map` frame:

```bash
ros2 topic pub --once /car_sando/term_goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}'
```

Wait for SLAM to establish free space first. SANDO may reject an unknown or blocked route. The **0.3 m plane is a planning reference**, not the LiDAR height or an instruction to lift the car. The physical car stays on its wheels. The upstream trajectory optimizer does not impose a full nonholonomic vehicle model; this prototype uses turn-first skid-steer tracking and bounded tracking error. Upstream UAV collision guarantees have not been established for this car/controller combination.

To regenerate the adapted planner from a different SANDO checkout, `scripts/prepare_sando.py --source /path/to/sando --output /new/separate/path` makes a new copy and refuses to overwrite an existing output or edit the original. Its source-pattern checks reject an incompatible implementation.

## 4. Optional: keep exploring reachable floor space

After verifying manual driving, SLAM and SANDO goal tracking:

```bash
ros2 launch slamdunk_car car.launch.py mode:=sando explore:=true
ros2 topic echo /car/exploration_status
```

SLAM maps are retained snapshots: a stationary map is not discarded solely because its capture stamp is old. New obstacle columns require fresh raw LiDAR and acquisition-time SLAM poses.

The bounded 2D frontier heuristic chooses observed-free viewpoints with clearance for the car, waits for a fresh observation after arrival, and retries other candidates after a stall. Remaining frontiers without an available sampled viewpoint report `blocked`, not `complete`. Completion requires four distinct fresh laser observations without a reachable observed-free/unknown boundary. New map evidence reopens selection. An exploration lease gates wheel commands; waiting, blocked, complete, a competing destination writer, or loss of the explorer's heartbeat withdraws motion requests. The stop latch is never automatically cleared.

Mission bounds are **X ±12 m, Y ±9 m**, matching the bundled house with its origin spawn. Map projection uses 0.2 m cells. Use the supplied world/spawn initially; changing them requires matching bounds in `mapping.py`, `sando_node.py`, `explorer_node.py` and `config/sando_car.yaml`. This is reachable-floor exploration, not a proof that every building surface or sealed room was surveyed. It assumes a flat supporting floor and does not detect cliffs, stairs, or traversable slopes.

## Topics and mapping scope

- `/car/odom`: wheel odometry, `odom -> base_link` TF from Gazebo DiffDrive.
- `/car/scan`: 360° scan in `laser`, used by SLAM Toolbox.
- `/car/points`: 3D LiDAR cloud in `lidar3d`; used for low-obstacle checks.
- `/car/imu`: IMU measurements. Published for recording; **not fused into wheel odometry** in this initial package.
- `/car/rgb/image`, `/car/rgb/camera_info`, `/car/depth/image`: RGB-D streams for later reconstruction.
- `/map`: SLAM Toolbox `OccupancyGrid`; SLAM owns `map -> odom` TF.
- `/car/map_odom`, `/car_sando/state`, `/car_sando/occupancy_grid`, `/car_sando/unknown_grid`: optional SANDO adapter outputs, all in `map`.
- `/car_sando/term_goal`, `/car_sando/goal`: high-level destination and trajectory setpoint.
- `/car/cmd_vel_teleop` or `/car/cmd_vel_sando` -> drive guard -> `/car/cmd_vel`.

Navigation mapping is **2D SLAM with loop closures**. Publishing 3D sensor clouds does not produce a persistent colored 3D model or Splatfacto reconstruction. RGB/depth/calibration, timestamped camera poses and reconstruction remain a separate stage. The old maps/drone dashboard is not modified; in SANDO mode its existing read-only bridge can be pointed at `/car_sando/occupancy_grid` and `/car/map_odom` with `map_frame:=map use_sim_time:=true`.

All nodes use `/clock` and the same ROS domain. Never add another publisher for `odom -> base_link` or `map -> odom`. Restart the launch after resetting Gazebo/SLAM rather than mixing old goals/history with a new clock. Bypass commands sent directly to `/car/cmd_vel` do not pass through the guard. Its watchdog requires the guard process to remain alive; it is not a hardware emergency-stop system or a protection against a SIGKILL of the entire command process.

## Verification

Creation-time checks on macOS:

```bash
python3 -m unittest discover -s test -v
python3 -m compileall -q slamdunk_car scripts launch
bash -n run_gazebo.sh
```

These check control limits/freshness/braking, planar tracking, conservative map projection, exploration states and model/package structure. They do not replace ROS/Gazebo tests.

On the Ubuntu Gazebo machine:

```bash
gz sdf -k models/car/model.sdf
gz sdf -k worlds/house_car.sdf
# With the default teleop launch running, in a sourced second terminal:
python3 scripts/runtime_smoke.py
```

The smoke test waits for real sensor/map messages, requests a short slow movement, checks wheel odometry, then checks zero commands after request expiry. Keep the spawn area clear and inspect chassis motion/contact in Gazebo too. Inspect narrow doorways, turns, SLAM loop closure, low obstacles and pause/resume before enabling exploration.

**Runtime limit:** this Mac has no ROS, Gazebo or Linux Gurobi runtime. The physics/sensor launch and adapted C++ planner have not been built or executed here. Jazzy compatibility of the copied planner and pinned decomposition/message packages must be confirmed by the Linux build; build failures should be resolved before calling SANDO integration validated.

References: [Gazebo Harmonic DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html), [ROS/Gazebo bridge](https://gazebosim.org/docs/harmonic/ros2_integration/), [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox). Upstream sources and licenses are recorded alongside the copied planner/dependencies.
