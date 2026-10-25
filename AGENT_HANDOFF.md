# SLAMdunk agent handoff

Updated: 2026-10-24. Repository snapshot: `517f851` (`sims`).

This is the starting context for another agent working on this project. It summarizes checked-in code, documentation, and Git history. It does not recover conversations that are absent from this checkout. Goals attributed to the mission brief below come from the repository's existing design notes; unresolved product choices should be confirmed with the team. Read the actual implementation before modifying a behavior.

## Mission and priorities

Build a simulated, semi-autonomous disaster reconnaissance drone that helps an operator inspect an indoor environment and see a live 3D map. Reuse MIT ACL SANDO's navigation and collision-avoidance pipeline instead of implementing a new planner or SLAM system for the first milestone.

The intended experience combines live geometric mapping, a drone camera, operator guidance, Explore Selected Area, Return Home, and eventually a real Stop/Hold capability. Autonomous coverage is a stretch goal. The implemented autonomous behavior is configured waypoint navigation; it is not frontier exploration or a coverage algorithm.

Photorealistic reconstruction is a later, offline stage using recorded RGB and verified camera poses with Nerfstudio/Splatfacto. It must remain outside the planner/control loop. A live occupancy visualization and an offline Gaussian splat solve different parts of the mission.

The immediate engineering priority is a verified end-to-end simulator demonstration. Much of the integration code exists, but a working ROS/Gazebo flight has not been established by the checks recorded here.

## What has been done

| Work | Current result | Evidence and limits |
| --- | --- | --- |
| SANDO source investigation | Audited sensors, mapper, commands, timestamps, TF, and reconstruction entry points | [Sensor audit](docs/sando-sensor-reconstruction-entrypoints.md); source findings, not a successful simulation run |
| Upstream runtime pinning | Added `vendor/sando` submodule at `1a738df919f9abe60cee3c1afbecd72579da8f6f` and a selected reference slice | [Integration setup](integration/README.md); runtime submodule is currently uninitialized in this checkout |
| Live-map backend | ROS 2 map/odometry adapter, bounded read-only WebSocket stream, legacy viewer, Python synthetic demo | [Bridge guide](integration/slam_dunk_bridge/README.md); tested without a real ROS runtime |
| Operator dashboard | Live and History screens, 3D/map interaction, bounded in-memory replay, status, camera panel, steering UI | [UI guide](integration/slam_dunk_bridge/OPERATOR_UI.md); demo and live data are explicitly distinguished |
| Operator command integration | Separate command sidecar and ROS mission supervisor with held steering, leases, ownership checks, and waypoint handoff | [Control guide](integration/slam_dunk_bridge/SANDO_CONTROL.md); planner/controller behavior remains unvalidated |
| Stop/Hold investigation | Detailed design for native planner braking, hold, cancellation, and control epochs | [Control design](docs/operator-push-stop-ros-design.md); physical Stop/Hold is not implemented |
| House environment | Generated furnished eight-room Harmonic world, moving obstacles, static baseline, floor plan, and layout metadata | [House guide](simulation/house/README.md); no drone or SANDO integration included |
| Automated verification | JS property/UI tests, Python unit/stub tests, real socket contract and HTTP smoke tests | [Recorded test report](integration/slam_dunk_bridge/test/UI_TEST_REPORT.md); these do not establish actual flight or camera operation |

Git milestones: `d9ca44b` reference/runtime setup; `67cfcd3` live bridge; `56182aa` dashboard, supervisor, and camera integration; `517f851` house simulation. PR merges appear as `6eb63da` and `63ac30f`.

## Architecture and ownership

```text
CURRENT SANDO PATH: Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic

Mid360 LiDAR (or D435 depth cloud) -> ACL mapper -> SANDO planner
                                           |           ^
                              occupancy map |           | term_goal
                                           v           |
Odometry --------------------------> live_map_bridge    mission_supervisor
                                     :8765/stream      :8766/control
                                           |           ^
                                           v           |
                                      browser dashboard
                                           ^
D435 RGB -> external web_video_server :8081/stream -----+

SANDO -> /NX01/goal -> fake_sim in the supplied simulation

SEPARATE ENVIRONMENT: Gazebo Harmonic / Gazebo Sim 8
simulation/house/worlds/*.sdf -> house only; robot and ROS integration pending

FUTURE OFFLINE PATH
raw sensors + calibration + timestamped camera poses -> dataset exporter
                                                    -> Splatfacto -> splat asset
```

Keep command ownership explicit: the supervisor chooses destinations, SANDO chooses collision-avoiding trajectories, and a downstream controller executes setpoints. Browser code must not write actuator setpoints directly. The existing telemetry socket accepts no commands; the optional sidecar owns the separate command connection.

### Runtime source versus reference copies

- `vendor/sando`: real upstream runtime source once initialized. Follow its build instructions and nested dependency requirements, including Gurobi. Upstream planner/plugin changes belong here or in a maintained fork.
- `integration/sando_reference`: selected source copies for reading/adaptation. `COLCON_IGNORE` deliberately excludes this incomplete package from builds. Editing these copies alone does not change the runtime.
- `integration/sando_reference_manifest.json`: provenance for the copied reference files.
- Preserve upstream licenses and source notices; the root license does not relicense dependencies.

## Important implemented behavior

### Map and pose bridge

Defaults: `/NX01/occupancy_grid`, `/NX01/visual_slam/odom`, planning frame `map`, loopback port 8765, target at most 10 Hz, 20,000 displayed points, 2,000,000 input points / 64 MB cloud, and eight clients.

Protocol v1 sends JSON `map`, `pose`, and `status` messages. Each map message is a complete replacement, including an empty array that clears the map. Do not append snapshots as raw sensor observations. The bridge keeps the latest message per type; intermediate samples may be superseded, and slow clients time out. Sampling is by input index, not spatial voxel filtering.

Non-target frames require TF at the message timestamp; there is no latest-TF fallback. Invalid frames/geometry, zero or duplicate/out-of-order timestamps, and unavailable transforms are rejected. Source time rewind requires restarting the bridge. Keep clocks consistent; blanket `use_sim_time=true` is not a demonstrated fix for upstream clock issues.

Current map points are height-colored XYZ, not RGB. The displayed path is received pose history, not the planner's intended trajectory. Existing geometry remains visible with stale/disconnected labeling. Live failures never silently switch to synthetic data.

The upstream map is finite, filtered navigation occupancy, with clearing behavior. It is not a persistent, complete building reconstruction. History accumulates a bounded sample for viewing; old obstacles can persist, and eviction can remove old geometry. History is in memory and resets on refresh.

### Held steering and waypoint navigation

- Control is opt-in. The checked-in YAML has `enable_exploration: false` and `waypoints_json: '[]'`.
- Readiness requires valid configured waypoints, fresh map/pose in the planning frame, and admission as the sole destination publisher. The source freshness budget defaults to two seconds and accounts for source age at receipt.
- Forward/Back/Left/Right capture drone heading when the press is accepted. The goal is a rolling horizontal destination, normally 3 m ahead, preserving altitude. It is destination guidance, not direct velocity control.
- Browser renewal is every 100 ms; the wire contract requires a 500 ms lease. An independent monotonic watchdog expires it. Updates are bounded to 5 Hz and require at least 0.15 m position progress. Lookahead must exceed the configured goal-seen radius by more than 0.1 m.
- Release, focus/page loss, disconnect, cancellation, or lease expiry withdraws the manual override and resumes configured waypoint navigation when readiness permits. Reconnect requires a new press.
- Epochs, request IDs, lease IDs, sequence numbers, ownership, and cancel-before-ack handling prevent delayed messages from reviving released presses.
- Stale telemetry or publisher conflict withdraws authority, but the last SANDO goal may continue executing. This is a critical limitation, not a physical stopping mechanism.
- Publisher admission checks require exactly one matching endpoint stable for at least one second; they are not distributed mutual exclusion. Disable/remap other writers such as `goal_sender` and `goal_relay`.
- Live Pause is disabled (`pause:false`). Synthetic Pause/Resume only illustrates the intended interaction.

The older Stop/Hold design contains superseded one-click and release-to-brake proposals. Its opening update and the implemented control guide take precedence for held buttons: release resumes waypoint exploration.

### UI, camera, and settings

Live includes top view, steering, main 3D view, camera, speed/margin controls, battery, and flight time. History contains combined occupancy and replay. Real bridge telemetry currently supplies neither battery nor flight duration nor velocity extensions; unavailable values must stay unavailable.

Camera rendering accepts an external HTTP(S) image/MJPEG URL, normally supplied by `web_video_server`. No feed produces an offline placeholder. The panel releases resources when hidden/disconnected/disposed. An image-load event does not establish per-frame timestamps, FPS, or freshness. RGB recording and dynamic-object tracking are not implemented.

The clearance slider interpolates proposed `inflation_hgp` 0.45-0.90 m and `dyn_base_inflation_m` 0.10-0.30 m. These are proposed parameter values, not measured clearances or collision probabilities. SANDO reads the settings at initialization; there is no acknowledged runtime application. More inflation may make doorways infeasible. Speed preference is also not measured speed.

## House simulation and the compatibility gap

The standalone house targets Gazebo Harmonic / Sim 8, including Ubuntu 24.04 use. SANDO's current launch files and plugins target Gazebo Classic with ROS 2 Humble on Ubuntu 22.04. Classic plugins and `gazebo_ros` wiring cannot be loaded directly into Harmonic. A house world existing does not mean SANDO can fly in it.

The house is 24 x 18 m with 3.2 m walls and no roof, eight rooms, a 2.4 m central hallway, 1.6 m room door openings with 2.2 m headroom, and a 1.8 m west entrance. Suggested drone spawn is `(-11, 0, 1)` m facing +X. Flight height must leave clearance for the full drone body through doors; SANDO's forced-altitude setting needs deliberate configuration.

There are 188 static models / 341 static collision shapes, two circular patrol obstacles, and a rotating ceiling fan. The static world holds those mechanisms at zero velocity. Built-in Harmonic joint controllers drive the obstacles; these are repeatable constrained mechanisms, not mobile robot dynamics.

`generate_house.mjs` produces both SDF worlds, `layout.json`, and `floorplan.svg` and checks geometry, spawn/entrances, and conservative patrol swept volumes. Creation-time generation/layout/XML checks were recorded as passing. libsdformat, physics, rendering, and robot sensor operation still need validation on a Harmonic host.

Choose a simulator strategy explicitly before investing in integration: validate the existing Classic baseline first, then decide whether to adapt an environment to Classic or port the robot/plugins and bridge to Harmonic. No such port is present. The house alone supplies no drone, flight controller, mapper, or ROS topic bridge.

## Sensor and reconstruction findings to preserve

Expected namespace is `NX01`, but inspect the live graph before hardcoding names.

| Data/interface | Expected topic | Meaning |
| --- | --- | --- |
| Occupied map | `/NX01/occupancy_grid` | PointCloud2 navigation snapshot |
| Optional smaller map | `/NX01/downsampled_occupancy_grid` | Only available if upstream downsampling enabled |
| Unknown space | `/NX01/unknown_grid` | Unknown is not measured solid geometry |
| Raw LiDAR | `/NX01/mid360_PointCloud2` | Default navigation input |
| RGB and calibration | `/NX01/d435/color/image_raw`, `/NX01/d435/color/camera_info` | Preserve images and intrinsics |
| Depth and calibration | `/NX01/d435/depth/image_raw`, `/NX01/d435/depth/camera_info` | Verify depth units and dimensions |
| Depth cloud | `/NX01/d435/depth/color/points` | Topic name does not guarantee RGB registration |
| State / odometry | `/NX01/state`, `/NX01/visual_slam/odom` | Synthesized by fake_sim; not independent ground truth |
| Operator destination | `/NX01/term_goal` | PoseStamped destination; already in planner frame |
| Controller setpoint | `/NX01/goal` | `dynus_interfaces/msg/Goal`; do not confuse with destination |
| Transforms / time | `/tf`, `/tf_static`, `/clock` if present | Record with compatible QoS and verify clock alignment |

Both LiDAR and D435 are mounted in the active model; `--d435` changes mapper input rather than adding a camera. Actual RGB publication has not been validated here.

Source-audit limitations:

1. Default RGB is 720 x 540 and depth is 120 x 90. The plugin's shared image-buffer color path does not provide synchronized color registration and can yield black cloud colors. Capture RGB separately and use timestamped reprojection with calibration/extrinsics.
2. The upstream mapper uses latest TF rather than acquisition-time TF. Its output stamp is map publication time; the bridge cannot repair upstream fusion timing.
3. Camera and LiDAR stamping use different wall-time paths. Recording `/clock` alone does not synchronize them.
4. fake_sim follows planned setpoints and asynchronously updates Gazebo. Its TF, odometry, and rendered pose can differ. Accurate reconstruction needs verified rendered model/camera poses at capture time.
5. LiDAR defaults to accumulating five scans in sensor-local coordinates under one new timestamp, without per-scan motion compensation. Start accurate capture with one scan per message or implement and validate compensation.
6. Low depth resolution and moving obstacles complicate reconstruction. Start with a slow overlapping scan in a static room.

The future exporter should emit RGB images, CameraInfo intrinsics, and per-image camera-to-world matrices in `transforms.json`. For map-from-ROS-optical pose `T_map_optical`, the proposed Nerfstudio conversion is `T_map_optical * diag(1, -1, -1, 1)`; flip local camera Y/Z, not the world frame. Preserve map registration and any training parser centering/scaling.

No bag-to-Nerfstudio exporter, trained scene, persistent raw-cloud fusion, registered colored cloud, or splat viewer integration is implemented. The audit discusses optional colored point-cloud initialization; it is not automatic LiDAR supervision. Gaussian PLY export is a splat asset, not a conventional textured mesh. Recheck installed Nerfstudio/CUDA requirements before training; historical GPU estimates in the audit are not hardware validation for this project.

## Physical Stop/Hold remains separate work

Sending a current-position destination, canceling a solver, or stopping publication does not physically brake the vehicle. SANDO has no supplied operator Stop/Hold service in this integration. The proposed extension requires native planner modes and synchronization: latch authority/epoch, reject stale solve commits, compute a collision-checked feasible braking trajectory, account for latency, hold a fixed anchor with continuous setpoints as needed, and monitor obstacles. A downstream watchdog must cover process/controller failures.

The design's proposed `operator_navigate`, `stop_hold`, `control_status`, and `resume` ROS interfaces are not implemented just because they appear in the document. Existing sidecar `control_status` messages are not evidence that these proposed planner interfaces exist. Hardware flight, PX4 integration, physical braking, and indefinite safe hover are unvalidated.

## How to run the available pieces

Commands below are entry points, not evidence that dependencies are installed. Linux ROS commands require the appropriate sourced, built workspace. Use the linked component guides for full setup.

Initialize runtime source from the project root:

```sh
git submodule update --init --recursive vendor/sando
```

Follow `vendor/sando/README.md` for upstream dependencies, build, Gurobi, and simulation launch. Its `scripts/run_sim.py` lives under `vendor/sando` in this repository, not the project root. Selected copies in `integration/sando_reference/scripts` are not a complete runnable runtime.

Browser-only UI preview, from `integration/slam_dunk_bridge`:

```sh
python -m http.server 8080 --bind 127.0.0.1 --directory slam_dunk_bridge/web
```

Open `http://127.0.0.1:8080/`. No ROS or npm build is needed. This is synthetic UI data.

Python stream demo, from that same directory, in an environment with Python 3.10+:

```sh
python -m pip install 'websockets==13.1'
python -m slam_dunk_bridge.demo
```

Open `http://127.0.0.1:8765/?source=live`. This connects to a synthetic backend, not ROS. The dashboard at `/` defaults to its separate browser-only demo.

Real bridge, from the repository root on the Humble machine, after starting SANDO and the mapper:

```sh
source /opt/ros/humble/setup.bash
python3 -m pip install 'websockets==13.1'
colcon build --symlink-install --base-paths integration/slam_dunk_bridge
source install/setup.bash
ros2 run slam_dunk_bridge live_map_bridge
```

Open `http://127.0.0.1:8765/?source=live`. Pinning `websockets==13.1` matters: the code uses its explicit legacy server API. Use the same Python environment for dependency installation and ROS execution.

Optional steering, after configuring scene-specific waypoints and disabling competing destination writers, from the bridge package directory:

```sh
ros2 run slam_dunk_bridge mission_supervisor --ros-args \
  --params-file config/operator_supervisor.yaml -p enable_exploration:=true
```

Open `http://127.0.0.1:8765/?source=live&control=ws%3A%2F%2F127.0.0.1%3A8766%2Fcontrol`.

For camera, run a compatible `web_video_server` separately on loopback port 8081 and enter `http://127.0.0.1:8081/stream?topic=/NX01/d435/color/image_raw&type=mjpeg` in the dashboard. Verify real topic publication first. Map, control, and video ports need separate forwarding when using another computer. These are development endpoints without production authentication/TLS; prefer loopback and SSH tunnels.

Harmonic environment only, from the project root on a compatible host:

```sh
bash simulation/house/run_house.sh
bash simulation/house/run_house.sh --static
```

## Verification evidence and current environment

The [2026-10-24 test report](integration/slam_dunk_bridge/test/UI_TEST_REPORT.md) records 12,903 named/parameterized JS cases, 26 Python test methods with generated scenarios, a cross-language real-socket control smoke session using a fake goal sink, and HTTP/telemetry smoke coverage. These counts include repeated invariants over generated inputs; they are not thousands of independently validated flight behaviors.

Recorded commands, from `integration/slam_dunk_bridge`:

```sh
node test/run_ui_tests.mjs
python -m unittest discover -s test -v
node test/control_contract_smoke.mjs python
python test/smoke_dashboard.py 8794
```

Those results are historical evidence, not fresh execution during this handoff. Tests use ROS stubs and DOM/canvas doubles. Actual browser visual review of the latest changes was not completed after Computer Use could not reliably determine Chrome's URL. CSS/layout, native browser behavior, accessibility, rendering performance, and actual camera delivery remain to be checked.

Current handoff inspection is on Windows/PowerShell. `Get-Command` resolved `wsl.exe` but did not resolve `python`, `node`, `ros2`, `docker`, or `gz`; WSL presence does not establish an installed or usable Linux distribution. Earlier documentation records Python/Node test versions, so do not assume those tools are still on the current shell PATH. No application tests or simulator launches were performed while writing this document.

`git submodule status` currently reports a leading `-` for `vendor/sando`, meaning it is uninitialized. Older audit statements that all submodules were initialized describe an earlier source-inspection context and must not be applied to this checkout.

## Next work and acceptance criteria

1. **Establish a runtime host and baseline.** Confirm access to the documented Classic/Humble environment, dependencies, Gurobi, and actual simulator launch. Initialize the pinned source and inspect live topic names, types, QoS, frames, timestamps, and publishers. Acceptance: ordinary SANDO navigation and raw RGB/cloud publication demonstrated.
2. **Validate real telemetry in the UI.** Run the bridge against that baseline. Confirm frame alignment, map replacement/empty clear, stale labels, reconnect behavior, and bounded CPU/network use. Measure actual end-to-end latency rather than quoting the configured 10 Hz target as latency.
3. **Validate command integration deliberately.** Configure environment-specific waypoints/altitude/bounds, establish sole publisher ownership, then test hold, renewal, release, disconnect, expiry, stale sources, and conflicts. Acceptance: correct destination handoff and honest fault reporting; do not present these tests as braking validation.
4. **Resolve house integration.** Select Classic adaptation or Harmonic port with the team. Validate SDF/physics/rendering on its target simulator, insert a compatible robot, and verify sensor/controller/ROS behavior before claiming house mapping works.
5. **Finish mission features.** Explore Selected Area and Return Home need implemented, acknowledged request semantics and waypoint/room handling. The house layout can inform manual labels, but no room-selection integration exists. Real Stop/Hold requires the planner work above. Coordinate UI capability changes with backend capability reporting.
6. **Build accurate capture before training.** Record raw RGB/depth/LiDAR, calibration, static/dynamic TF, and verified capture poses with timing/QoS checks. Then implement dataset export, capture a static-room dataset, and validate offline splats against held-out images and known metric geometry.

No end-to-end simulator demonstration, house/SANDO compatibility, persistent reconstruction, mission-complete coverage, or hardware readiness should be claimed until its corresponding runtime evidence exists.

## Code navigation

| Location | Responsibility |
| --- | --- |
| `integration/slam_dunk_bridge/slam_dunk_bridge/core.py` | Cloud decoding, transforms, latest-message store, map serialization |
| `integration/slam_dunk_bridge/slam_dunk_bridge/node.py` | ROS map/odometry subscriptions, TF, timestamps, bridge parameters |
| `integration/slam_dunk_bridge/slam_dunk_bridge/transport.py` | HTTP assets and read-only telemetry WebSocket |
| `integration/slam_dunk_bridge/slam_dunk_bridge/control.py` | ROS-independent steering supervisor and lease/waypoint logic |
| `integration/slam_dunk_bridge/slam_dunk_bridge/command_server.py` | Bounded command WebSocket server and origin/connection handling |
| `integration/slam_dunk_bridge/slam_dunk_bridge/supervisor_node.py` | ROS supervisor wrapper, source freshness, publisher admission, goal output |
| `integration/slam_dunk_bridge/config/operator_supervisor.yaml` | Opt-in configuration and empty default waypoint list |
| `integration/slam_dunk_bridge/slam_dunk_bridge/web/` | `app.mjs` UI, `adapter.mjs` connections/control, `model.mjs` state/history, `scene.mjs` rendering, `camera.mjs` camera lifecycle, `demo.mjs` synthetic behavior |
| `integration/slam_dunk_bridge/test/` | Unit/property/UI-double, ROS-stub, socket, and smoke tests |
| `simulation/house/generate_house.mjs` | House geometry and artifact generator |
| `simulation/house/worlds/` | Checked-in moving and static SDF worlds |
| `docs/` | Source audit and proposed planner control design |

For the next agent: begin with this handoff, inspect the guide for the component being changed, and check current Git/runtime state. Keep implemented behavior, design proposals, synthetic demonstrations, and measured runtime results explicitly separate in documentation and UI.
