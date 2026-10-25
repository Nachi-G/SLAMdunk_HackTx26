# Continuous frontier exploration

The optional mission supervisor can choose destinations from sensor observations
instead of following a repeating waypoint list. It keeps selecting known-free
viewpoints near unexplored space. SANDO still owns trajectory feasibility and
collision avoidance. This is a bounded frontier heuristic, not FUEL or a globally
optimal coverage algorithm. No UI or upstream planner files are changed.

## Enable in Gazebo

Build/source this package using [README.md](README.md). Keep the existing SANDO,
mapper, sensor, and supervisor in the same ROS domain. Copy and edit
`config/frontier_supervisor.yaml`:

- Set the finite mission `bounds` inside SANDO's planning bounds. The example
  20 x 20 m region is not an inferred building boundary. Grid size is capped;
  reduce the region or increase voxel size if startup rejects the configuration.
- Verify the raw LiDAR topic, sensor frame, range, source timestamps, and TF.
  Default topic: `/NX01/mid360_PointCloud2`. An empty
  `exploration_sensor_origin_frame` uses the cloud's own sensor frame. For a
  cloud already in `map`, explicitly configure its real sensor-origin frame.
  The map origin is not accepted as a sensor origin.
- Keep clocks consistent with the pinned SANDO setup. Current plugins have
  wall-time stamping paths; changing only `use_sim_time` is insufficient.
- Begin with a stationary scan. The sensor must establish observed free
  clearance around the pose before autonomous goals are admitted. This node
  does not take off or bootstrap by assuming unknown space is free.
- The upstream simulated Mid-360 accumulates five historical sensor-local scans
  into one cloud by default. Configure `accumulate_scans` to **1** in the runtime
  sensor Xacro, or implement acquisition-time motion compensation. Timestamped
  TF cannot repair already accumulated moving-sensor data.
- Match vehicle clearance, altitude, arrival radius, and mission bounds to
  SANDO. Clearance rounds up to a conservative whole-voxel cube envelope.
- Disable/remap **all other destination writers**, including `goal_sender` and
  `goal_relay`. This is another mode of the same supervisor, not a second writer.

From this package directory in a sourced ROS workspace:

```sh
ros2 run slam_dunk_bridge mission_supervisor --ros-args \
  --params-file config/frontier_supervisor.yaml \
  -p enable_exploration:=true
```

The config defaults to disabled. It is also installed under
`share/slam_dunk_bridge/config/`. Existing waypoint-mode defaults are unchanged.

### Horizontal or 3D coverage

The example uses a horizontal slice at 2 m, matching SANDO's default
`force_goal_z=true` and `default_goal_z=2.0`. Completion explicitly reports
**horizontal slice**, not whole-building volumetric coverage.

For full 3D frontier search, set `exploration_3d: true`, configure all three
mission bounds, and **also disable `force_goal_z` in the actual SANDO runtime
config**. Verify the altitude limits. The explorer then uses six-connected 3D
free-space searches; it cannot override SANDO's forced altitude by itself.

## Map and destination behavior

Finite raw sensor rays are transformed using acquisition-time TF. Interiors
provide free evidence; endpoints provide occupied evidence. Hits win within
one scan. Areas not traversed remain unknown. The bounded voxel map persists
in memory as the drone moves; later rays can clear obstacles that move away.
Invalid, duplicate, stale, out-of-order, and missing-TF observations do not
refresh readiness. Points beyond configured range are discarded rather than
treated as empty max-range observations. Only the latest pending scan is kept.

The explorer finds frontiers of the connected observed-free component containing
the drone. It ranks a bounded set of known-free, clearance-checked viewpoints
using visibility of unknown cells and travel distance. A conservative free-cell
graph estimates reachability, while SANDO makes the actual trajectory decision.
One active goal is retained while progress continues. After arrival it waits for
a new sensor observation; arriving alone never changes coverage labels.

Stalled or unproductive viewpoints have cooldowns and bounded retries. A genuine
map change reopens failed candidates. Remaining frontiers with no viable sampled
viewpoints produce **blocked**, not completion. After frontier exhaustion,
several distinct fresh observations must confirm it before **complete** is
reported. New frontiers reopen exploration. Unknown regions behind sealed walls
remain unknown and are included in status counts, not marked surveyed.

The navigation mapper's clouds cannot safely supply free space by complement.
At pinned mapper commit `61b9cff690419c9fb309dfb80cbbce6fdd80d9e4`,
`PopulateUnknownPointCloudMsg()` filters by altitude and radial limits, and the
occupied output filters ground. Exploration therefore uses raw rays instead.
The existing occupied map is still a required fresh navigation input.

## Supervisor and status

The existing exclusive-writer check and manual steering lease remain in force.
Manual steering suspends frontier selection without recording a failed attempt;
release/expiry/disconnect reselects using the updated map and pose. Stale inputs
withdraw new autonomous goals.

The existing `control_status.reason` carries the current explanation. An additive
`exploration` object reports state, goal, map revision, scan sequence, frontier
and reachable counts, unknown/free/occupied voxel counts, failed viewpoints, and
completion scope. The top-level control protocol is unchanged.

**Completion stops issuing destinations; it is not braking, landing, or physical
Stop/Hold.** If a manual goal was active and no replacement is possible, status
warns that the prior SANDO destination may remain active. The existing physical
Stop/Hold limitation remains.

## Limits and verification

- The map persists for one process lifetime, not in a saved SLAM database. There
  is no loop-closure correction of historical evidence. Upstream localization,
  timing, and range geometry must be validated.
- This models all-direction range sensing for the mounted LiDAR. Camera yaw,
  limited-FOV view planning, image overlap, and reconstruction quality are not
  optimized. A forward depth camera may stall while looking away from a frontier.
- Resource limits include bounded grid size, at most 1,200 sampled rays per scan
  by default, an atomic 100,000 traversed-cell budget per scan, 2 Hz latest-only
  scan processing, finite viewpoint search, and at most 1 Hz goal selection when
  no goal is active. Graph/viewpoint searches run in a separate latest-only
  worker on immutable grid snapshots, outside the lease watchdog and ownership
  lock. Old map/pose/control-generation results are discarded. These are not
  hard real-time guarantees; measure watchdog/goal latency on the target computer.
- Battery reserve, return-to-base, communications-aware exploration, takeoff,
  landing, and loss-of-link flight procedures are not implemented. Independent
  mission/controller safeguards must enforce those limits.

```sh
python3 -m unittest discover -s test -v
node test/run_ui_tests.mjs
node test/control_contract_smoke.mjs python3
```

Deterministic tests cover ray evidence, persistence, boundary clipping, stale
inputs, sequential goals through a doorway, completion confirmation, residual
unknown, blocked retries, map-driven reopening, altitude/clearance, and manual
handoff. ROS stubs check latest-only raw cloud handling and timestamped TF.
These are not live Gazebo or physical flight validation.
