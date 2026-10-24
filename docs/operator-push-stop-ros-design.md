# Operator-directed exploration and Stop/Hold in SANDO

Source audit: 2026-10-24; SANDO checkout `1a738df919f9abe60cee3c1afbecd72579da8f6f`.
This is a proposed integration design, not implemented or runtime validated control behavior.

Update: the approved held-direction interaction now resumes autonomous waypoint
exploration on release or lease expiry, rather than release-to-brake. The
[implemented operator integration](../integration/slam_dunk_bridge/SANDO_CONTROL.md)
documents the separate leased supervisor, camera stream and test coverage.
The explicit physical Stop/Hold design below remains unimplemented; live Pause
is disabled rather than claiming that destination publication physically stops
the vehicle. Earlier one-click/release-to-brake proposals below are superseded
for directional UI buttons.

## Decision and mission fit

Use operator-directed destinations to guide SANDO, with an explicit braking/hold mode inside the planner node. Add a small ROS 2 mission supervisor between the browser backend and SANDO. SANDO continues choosing collision-avoiding trajectories; the supervisor chooses the destination and owns command priority.

The current [HackTX Ideas mission brief](https://docs.google.com/document/d/1njPtANW2rjWx5Q6OLFITjIoHeWGfc0Q0N86iRqBrWpY/edit) calls for a simulated, semi-autonomous disaster reconnaissance drone, live 3D mapping, Explore Selected Area, Return Home, and Stop/Hold. Autonomous coverage is a stretch goal. A sequence of operator-selected waypoints fits the immediate mission; a new autonomous exploration algorithm is unnecessary for that milestone.

The [SANDO paper, Sections III, IV-C, and VIII-E](https://arxiv.org/html/2604.07599v1) describes goal-directed global planning, local trajectory optimization, and conservative treatment of unknown space. It supports using a destination inside an unseen room while advancing through observed safe subgoals. Its safety result applies within a planning horizon under stated assumptions; it does not establish recursive feasibility. Therefore a custom stop trajectory needs its own feasibility checks, and an indefinitely motionless hover cannot be guaranteed safe against moving obstacles.

Deterministic here means explicit command ownership, ordering, cancellation, and reported outcomes. It does not mean an identical trajectory on every run, zero physical stopping distance, guaranteed room entry, or a hard real-time response deadline on ordinary ROS 2/Linux.

## What the checkout actually supports

| Existing interface/code | Observed behavior and implication |
|---|---|
| `src/sando/sando_node.cpp:147`, `:951` | `term_goal` accepts `geometry_msgs/msg/PoseStamped`. This is the destination input. The callback reads XYZ, checks altitude, and starts replanning. It does not transform `header.frame_id` or use the supplied orientation. |
| `src/sando/sando.cpp:1817` | Goals within 0.1 m of the previous goal are ignored. During TRAVELING/GOAL_SEEN, a changed goal updates the destination smoothly without clearing the active plan. This enables redirection, but is not an immediate stop. |
| `src/sando/sando_node.cpp:1534` | `goal` publishes `dynus_interfaces/msg/Goal`: position, velocity, acceleration, jerk, yaw and yaw rate. This is an actuator/controller setpoint channel, not an operator destination. Default `dc=0.01` requests 100 Hz publication. |
| `src/sando/sando_node.cpp:152` | Replanning timer requests a cycle every 10 ms. Effective response includes scheduling and solve time; 100 Hz is not a guaranteed new-plan rate. |
| `src/sando/sando.cpp:1340`, `:1592` | New setpoints are appended to an existing plan; the last remaining setpoint is retained. Canceling a timer does not cancel a trajectory already computed or a callback already executing. |
| `src/sando/sando_node.cpp:158`, `:1012` | `goal_reached` is an Empty event, with its timer only created when benchmarking is enabled. It cannot reliably identify the completed operator request. |
| `src/sando/gurobi_solver.cpp:4128` | The solver has final position/velocity/acceleration constraints. This is useful machinery for a braking trajectory, but there is no operator Stop/Hold API connecting it today. |
| `src/sando/gurobi_solver.cpp:24` | `stopExecution()` terminates a solver search. It does not brake the drone or replace queued setpoints. |
| `src/sando/fake_sim.cpp:234`, `:297` | Simulation copies commanded state and updates Gazebo pose; TF extrapolation is capped at 100 ms. Visual stopping in this simulator does not validate physical braking. |
| `src/sando/sando_node.cpp:1538` | Hardware publication is suppressed at GOAL_REACHED. New HOLDING behavior must not accidentally inherit this suppression if the downstream controller needs continuous commands. |
| `hacktx-transfer/integration/slam_dunk_bridge/slam_dunk_bridge/transport.py:30` | Current WebSocket stream is output-only and closes clients sending messages. Navigation commands require a new backend command route. |

Names assume namespace `NX01` and the simulation planning frame `map`; verify against the running graph. Hardware may use `world` and additional transforms. `convert_goal_to_cmd_vel` targets ground robots; it is not the drone control path.

## ROS data flow

```text
Browser: click destination / push / room / return / stop
                |
        backend command route
                |
        mission_supervisor (ROS 2)
        validates input, owns active request, sequences room waypoints
                |
       +--------+----------------------+
       |                               |
  accepted destination            Stop/Hold request
  /NX01/term_goal                 NEW /NX01/stop_hold service
       |                               |
       +---------- SANDO node ---------+
                   NORMAL / BRAKING / HOLDING / AVOIDING
                   one authoritative output publisher
                              |
                   /NX01/goal (dynus Goal)
                              |
                   fake_sim in current simulation
                   actual controller bridge on hardware

state/odom + map + control status -> supervisor/backend -> browser
```

Keep mapping and recording running during hold. Photorealistic reconstruction remains outside the control loop.

## Push, room entry and return semantics

* **Go to point:** convert a map click to a metric 3D destination, validate finite coordinates, bounds, altitude and map freshness, then publish the accepted target. `force_goal_z=true` currently overrides requested altitude to 2 m; configure this deliberately for indoor rooms.
* **Push in a direction:** treat one button click as a bounded displacement. At acceptance, capture pose and heading once. For a horizontal body-relative displacement `d=(dx,dy)`, use `target_xy = position_xy + Rz(yaw) d`. Preserve the chosen flight height. Declare ROS body conventions (+X forward, +Y left, +Z up); convert any browser axes before sending. A target sent to the current callback must already be in its planning frame.
* **Held direction button:** optional later feature with a short command lease, bounded update rate and explicit release-to-brake. Do not recompute a click's target on every state update: that creates a perpetually moving goal. Repeated request IDs must not create repeated displacements.
* **Explore room:** resolve the selected room into an approach point, a doorway crossing, and an interior observation point; send them sequentially. A single interior goal can choose another route, so it does not enforce a specific doorway. Doorway dimensions must admit the drone footprint plus configured inflation. Occupancy alone does not provide room names; use manual room labels/portals for the MVP. Report blocked/no-progress instead of forcing traversal through unknown or occupied space.
* **Return:** store a valid initial airborne staging pose and navigate to it through SANDO using the current map. This is a return destination, not reverse playback, landing, or a guarantee that the route remains open.

Do not disable unknown-space inflation or free occupied target voxels merely to force progress. Reaching an observation point is not equivalent to scanning an entire room. Current terminal-goal orientation is ignored; explicit scan yaw/coverage would need a separate extension.

## Proposed interfaces, all new except term_goal/state/goal

[ROS 2 actions](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Basic/About-Actions.rst) support feedback and cancellation, making them suitable for a navigation request that lasts seconds. An action wrapper alone does not implement physical cancellation in SANDO.

| Interface | Proposed contract |
|---|---|
| `/NX01/operator_navigate` action | Goal: request ID, target PoseStamped, mode, room/waypoint sequence if applicable, expected control epoch. Feedback: request ID, phase, distance, speed, planner health and state age. Result: reached, blocked, preempted, canceled or fault. |
| `/NX01/stop_hold` service | Short, idempotent request to latch BRAKING, increment control epoch, cancel room sequencing and invalidate previous navigation authority. Reply means accepted/latch applied, not physically stopped. |
| `/NX01/control_status` topic | Epoch, active request, NORMAL/BRAKING/HOLDING/AVOIDING/FAULT, measured speed, hold anchor, age and failure reason. |
| `/NX01/resume` service | Explicitly release hold using the current epoch. Resume/new navigation starts from fresh state and replans; do not replay old setpoints. |

Supervisor is the sole destination writer. Disable or remap demo goal_sender and direct goal_relay so they cannot overwrite it. Retain the current volatile reliable goal-input behavior; use shallow queues for frequently replaced setpoints and bounded command queues. Reject expired commands. Reliable delivery alone is not cancellation or freshness.

Latch stop until an explicit resume/new-go authorization referencing the current epoch. A delayed Go from before Stop must be rejected, including one arriving after the service reply. Define one active operator lease; loss of that lease triggers braking rather than leaving a held-button command active.

## Implementing actual Stop/Hold

1. **Latch authority quickly.** The stop callback changes control mode and epoch under synchronization. It must not wait for the ongoing Gurobi solve. Serialize the mode switch with setpoint publication so ordinary navigation cannot publish after the latch. Distinguish acceptance, braking active, and physically held in telemetry.
2. **Invalidate old planning work.** Every solve captures the epoch and target version at start. At `appendToPlan()`, compare versions under the same synchronization used to commit a plan; reject obsolete results. Solver cancellation is an optimization, not the authority mechanism. A mutex around plan data alone cannot distinguish an old result from a current one.
3. **Generate a feasible brake.** From fresh measured position, velocity and estimated acceleration, find a nearby terminal state with zero velocity and acceleration. Respect configured velocity/acceleration/jerk limits, observed free space, drone inflation and dynamic corridors. Braking generally advances along the current motion; commanding the current position with zero velocity instantly is a discontinuous reference, not a validated brake.
4. **Cover brake-generation latency.** Account for motion during request transport, scheduling, state age, computation and controller response. Retain only a checked executable prefix while a new brake is generated. For a stronger stopping bound, continuously maintain a valid braking fallback while navigating; arbitrary planner stalls cannot be handled by generating a brake only after Stop. If no feasible fallback exists, report FAULT and use an explicitly defined controller contingency; do not claim the drone is held.
5. **Hold a fixed anchor.** Once speed and position remain within tolerances for a dwell period, switch to HOLDING. Continue publishing the hold anchor with zero velocity/acceleration/jerk and fixed yaw as required by the downstream controller. Do not reset the anchor to every newly measured position, which would allow drift.
6. **Keep obstacle monitoring.** Hold means cancel the exploration task. If the hold region becomes threatened, allow reported safety evasion while keeping the mission paused. Start with the existing optional hover-avoidance code as a reference, but integrate it with the new mode and epoch rules; it is disabled by default and is not an unconditional safety guarantee.

For intuition only, straight-line braking with initial acceleration zero and acceleration bound `a` has `t=v/a` and `d=v²/(2a)`. Add latency distance `v*latency`; jerk limits, nonzero initial acceleration, tracking error and obstacles can increase the required distance. These equations are not a collision-checked stop planner. Start indoor tests at a deliberately reduced speed after measuring corridor clearance; current simulation limits are 5 m/s and 20 m/s² per configured formulation.

Use separate callback execution capacity for control/status and heavy solving, plus explicit synchronization. [ROS 2 executor documentation](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Executors.rst) explains callback scheduling; a multi-threaded executor alone does not guarantee priority or bounded latency. A downstream watchdog must handle planner-process loss separately.

For future PX4 hardware, audit the real controller bridge: it is not supplied by the current simulator launch. [PX4 Offboard documentation](https://docs.px4.io/main/en/flight_modes/offboard) requires continuing proof-of-life; MAVLink setpoint streaming and native ROS 2 OffboardControlMode have different heartbeat arrangements. Stopping the SANDO publisher is not a generic hover command.

## First experiment and implementation order

First verify ordinary retargeting in the existing interactive simulator. On a sourced Ubuntu ROS 2 Humble workspace, inspect topic types and publishers:

```bash
ros2 topic info -v /NX01/term_goal
ros2 topic info -v /NX01/goal
ros2 topic echo /NX01/state --once

# Example destination in map coordinates; choose coordinates clear in your scene.
ros2 topic pub --once /NX01/term_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0, z: 2.0}, orientation: {w: 1.0}}}"
```

That command demonstrates existing destination control only. There is no equivalent existing stop service command.

Then implement the supervisor/action/status contract, the native SANDO brake/hold mode and epoch checks, and finally the browser controls and room waypoint sequences. Update the active runtime source, not only `hacktx-transfer/integration/sando_reference`, which is an excluded reference copy.

Validate: redirect during flight; Stop while solving; delayed/replayed Go after Stop; repeated Stop; resume; lost operator connection; stale state/map; no feasible braking path; an obstacle entering the hold region; room doorway blockage; return with a changed route. Record command acceptance, first braking setpoint, measured settling time/distance and minimum clearance. Verify old-epoch plans never commit and room sequencing never restarts during hold. Repeat physical braking validation in a dynamics-based simulator/controller before drawing hardware conclusions.

This Windows host has no `ros2` or `docker` executable on PATH. The source and mission support the design above; transport latency, braking performance, sensor freshness and collision behavior remain unmeasured.
