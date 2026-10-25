# PX4 / SANDO perception inspection

Inspected existing files on 2026-10-24. Existing configurations were not edited.
No flight control messages, arming commands, or mode changes were issued.

## Confirmed from files

- Forest world: `gazebo_worlds/hacktx_forest.sdf`, world name `default`.
- World includes Physics, UserCommands, SceneBroadcaster, Sensors (ogre2), IMU,
  AirPressure, NavSat, and Magnetometer systems.
- Standard PX4 `x500` includes `x500_base`; neither contains a LiDAR.
- PX4 also supplies `x500_lidar_2d` with `lidar_2d_v2`, a GPU LiDAR with
  1080 horizontal samples over 270 degrees, 30 Hz, 0.1–30 m range.
  Its sensor has `always_on=0`. This alone does not establish the cause of
  missing output; active model, subscriptions, and rendering logs are needed.
- The forest world contains no embedded LiDAR or X500 model: the running vehicle
  must be spawned separately. Historical topic names do not establish which
  vehicle model is currently active.
- `/opt/ros/jazzy/share/ros_gz_bridge` is absent, and dpkg did not find installed
  ros-jazzy-ros-gz packages in this environment.
- `sando_nx01.yaml`: v_max=1, a_max=1, j_max=2, dynamic_constraint_type=L2.
  The YAML defaults use_global_pc=true; the handoff command overrides it false.
- SANDO fake_sim occupancyMapCallback at sando_node.cpp:2078 converts cloud
  contents directly to PCL and updates the map. It performs no TF conversion,
  frame validation, or timestamp validation. Input must already be in the
  same ENU frame and origin as `/NX01/state`.
- The existing state bridge stamps receipt time, not the PX4 measurement time.
  Synchronization and PX4 local-origin alignment need validation before using
  its pose to transform sensor data.
- Existing trajectory preview only evaluates position/velocity on trajectory
  receipt. It is not a continuously timed control loop or flight bridge.

## Live verification limitation

The Codex sandbox exposes an isolated process view. `gz topic -l` fails with
`error in getifaddrs: Unknown error -1`. Therefore no assertion is made about
host simulation processes, live publishers, or sensor functionality.

## Next concrete action

From an ordinary WSL terminal (leave existing simulator terminals running):

```bash
cd ~/hacktx
bash diagnostics/inspect_perception.sh > diagnostics/perception_host.txt 2>&1
```

Review this output before choosing a sensor fix. It collects processes, Gazebo
endpoints, ROS connections, bridge availability, and existing Gazebo error logs.
It does not start/stop components or publish messages.

Then verify an actual sensor sample and active model SDF. Prefer a separate
workspace-owned model variant if a new sensor is needed; keep working PX4
models and world unchanged. A 2D scan alone does not establish 3D forest
clearance. For a 3D planner, use suitable vertical coverage or explicitly
restrict and validate the test scenario.

The eventual pipeline must check acquisition time and sensor extrinsics,
transform points into the verified SANDO ENU origin using pose at acquisition,
and reject stale data. Initially publish to a separate preview cloud topic.
Validate multiple known tree positions before replacing the synthetic source.
Unknown or unobserved space must not silently become certified free space.

Offboard implementation and flight tests remain pending perception validation.
User approval is required before mode changes, arming, movement setpoints, or
any autonomous flight test.

## Active snapshot follow-up

`diagnostics/active_lidar/active_world.sdf` confirms x500_0 contains only
barometer, IMU, magnetometer, and GPS sensors. Neither queried LiDAR endpoint
has a publisher; both scan captures were empty. Thus a LiDAR must be attached,
not merely subscribed to. No evidence yet establishes a rendering failure.

Added separate `gazebo_models/x500_sando` variant: merged existing x500,
LiDAR link fixed 0.30 m above base_link, 720 x 32 rays, 360 degrees horizontal,
+/-30 degrees vertical, 0.3–25 m range, 10 Hz. Original PX4 models unchanged.
Both this variant (with SDF_PATH set to PX4 models) and the separate forest
sensor test world pass `gz sdf -k` with expected gz_frame_id extension warnings.
These are configuration checks, not runtime sensor or flight validation.

Run `python3 diagnostics/test_stationary_lidar.py` from ordinary WSL to test a
static probe at (0,0,2.3) in a copy of the forest, on a separate GZ_PARTITION.
The runner launches no PX4, sends no control commands, captures one scan,
and stops only its own server process group. Results are stored under
`diagnostics/stationary_lidar/`. A scan with finite returns is only a sensor
smoke test; tree association, frame/timestamp validation, ROS bridging, and
SANDO integration remain outstanding.

## Stationary sensor runtime result

Captured 23,040 rays (720 x 32), including 7,285 finite in-range returns.
Offline comparison against the copied forest's cylinder surfaces and bounded
floor matches 100% of valid returns within 0.10 m; median error is 0.0085 m.
Five trees have >=10 associated returns (model IDs 13,24,28,30,39).
This establishes actual forest geometry returns for the stationary fixture.
It does not establish collision-free paths or moving-drone perception.

The runner originally failed after saving its scan because protobuf JSON uses
camelCase fields and string Infinity values. This is corrected; the existing
scan was analyzed successfully without rerunning simulation. The original
runner was backed up as test_stationary_lidar.py.before_json_fix.

The first scan's worldPose is identity/zero despite the fixture being at
(0,0,2.3). Geometry validation uses the explicit fixture pose. Do not use that
reported field as a moving-sensor transform without further validation.

Added config/sando_lidar_preview_bridge.yaml: one-way PointCloudPacked to
PointCloud2, sensor frame retained, output /sando_perception/lidar/points_raw.
No planner input or PX4 input is bridged. It remains runtime untested until
ros-jazzy-ros-gz-bridge is installed on the WSL host.

## ROS bridge installed; isolated runtime test prepared

Confirmed /opt/ros/jazzy/share/ros_gz_bridge and bridge executable now exist.
Added diagnostics/test_ros_lidar.sh to start a separate stationary forest server
and one-way points/clock bridge, using unique GZ_PARTITION and ROS_DOMAIN_ID=87.
The validator requires ten clouds with expected sensor frame, increasing nonzero
acquisition timestamps, recent simulation clock, and matching forest geometry.
The preview uses forest_test_enu and retains source simulation timestamps;
it explicitly does not claim alignment with PX4's local map frame.
Only /sando_perception/points_preview is published by the validator.
The existing SANDO point-cloud input is not connected.

Local checks pass: Python syntax; padded big-endian organized PointCloud2
XYZ decoding; malformed buffer rejection; real saved scan round-tripped
through PointCloud2 still matches forest geometry above 99%.
Live Gazebo-to-ROS delivery is pending execution from ordinary WSL:
`bash diagnostics/test_ros_lidar.sh`. Results: diagnostics/ros_lidar/summary.json.

## First ROS delivery run: launcher failure isolated and corrected

The ROS bridge created both point-cloud and clock bridges, but the server log
contained only gz command help and no simulation startup. After sourcing ROS,
PATH resolves gz to /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz and GZ_CONFIG_PATH
contains only ROS vendor transport/message entries, so `gz sim` is unavailable.
This run received no clouds and provides no ROS sensor validation.

The runner now uses /usr/bin/gz with GZ_CONFIG_PATH=/usr/share/gz and excludes
ROS library paths only from the test simulator environment. The bridge retains
its normal sourced ROS environment. Verified locally: sourced ROS plus this
system command reports Gazebo 8.15.0 and recognizes simulator/headless options.
The runner also checks version before starting and monitors server/bridge
processes to abort promptly on early exit. Original runner backed up.
Run `bash diagnostics/test_ros_lidar.sh` again from ordinary WSL; runtime
point-cloud validation remains pending.

## Stationary Gazebo -> ROS runtime PASS

Ten successive clouds accepted, no rejections. Correct sando_lidar_link frame;
monotonically increasing simulation timestamps; clock minus stamp 0.003–0.008 s.
Median acquisition interval 0.1 simulation seconds, maximum 0.2 s.
All accepted returns matched copied forest visual geometry within 0.10 m.
Bridge publisher is RELIABLE/VOLATILE, validator sensor-data subscription matched.
Last preview contained 2,209 above-ground obstacle points in forest_test_enu.
Evidence: diagnostics/ros_lidar/summary.json. This is stationary validation only.

## Isolated SANDO integration test prepared

`bash diagnostics/test_sando_lidar.sh` launches the sensor copy, bridge, and
an additional SANDO instance on domain 88. Existing domain-0 PX4 and planner
are not connected. A fixture publishes static (0,0,2) state and verified live
sensor points in the test map frame, then goal (10,-4.44,2), beyond tree 28.
Cloud source simulation timestamps are retained; this SANDO fake_sim path
ignores cloud timestamps and plans using its normal wall clock. The fixture
checks cloud liveness and test clock alignment before publishing each map.
This is not a production solution for sensor/PX4 time alignment.

The trajectory checker samples at <=0.02 s, checks dynamics, altitude, segment
position/velocity/acceleration continuity, progress, and all 41 tree cylinders
expanded by a horizontal circumscribed drone radius and vertical half-height.
These sampled checks are not a formal continuous collision proof or flight test.
Local checker tests pass for open-space progress, direct-cylinder collision
rejection, and excessive-dynamics rejection. Runtime planner test is pending.
Results will be saved in diagnostics/sando_lidar/summary.json and planner.log.

## First isolated SANDO test: map delivery PASS, local optimization FAIL

SANDO received 261 validated real sensor clouds and the isolated goal.
READY CHECK confirms state=1 goal=1 map=1 kdtree=1; global paths are found,
but every local solve reports infeasible/unbounded. No trajectory was received.
Thus perception delivery into SANDO is demonstrated, but planning is not yet
functional for this test. Limits and working configuration have not been relaxed.

Prepared diagnostics/diagnose_sando_solver.sh. An opt-in model snapshot shim is
loaded only into the isolated planner process; it calls the original optimizer
and records the first failed model without altering constraints or return values.
No production binary/source was modified. Its C compilation and mock API
interposition/snapshot test pass. Actual solver replay must run on the WSL host:
the sandbox reports a different HostID, so its local Gurobi license cannot be
used here. The host script captures the model, stops its test processes, and
runs bounded Gurobi analysis to save conflicting constraints as conflict.ilp.
This follows Gurobi's documented ResultFile=.ilp diagnostic workflow.

## Solver conflict identified

Host replay proves the captured model infeasible. Gurobi IIS consists only of
vel_L2_seg2_cp0 and vel_L2_seg3_cp1 (two velocity constraints, no corridor
constraints). Completing the square gives disjoint feasible balls in d4:
radii 0.348889 and 0.174445 m, center separation 0.664691 m. For this pair
alone, at least 1.2701x more duration is necessary at unchanged speed limit.
This is evidence for inadequate timing in that captured solve, not proof that
all infeasibility will disappear with more time.

Added --longer-timing test option: fixed factor sweep 2.5..4.0, step 0.25.
Only isolated command-line parameters change; working YAML and safety limits
are untouched. Results go to diagnostics/sando_lidar_longer_timing/.
Run `bash diagnostics/test_sando_lidar.sh --longer-timing` from WSL.

## Longer timing planner test PASS

Isolated planner succeeds at factor 3.50 with the original speed/acceleration/
jerk limits and footprint unchanged. Verified one local polynomial trajectory:
duration 10.176 s; sampled speed 0.7711 m/s; acceleration 0.2760 m/s²;
jerk 0.1366 m/s³; altitude 1.85–2.00 m. Segment continuity errors <1.1e-7.
Sampled conservative cylinder separation >=1.7559 m across 41 tree cylinders.
Endpoint (2.907,-2.726,1.85) is still 7.298 m from the goal. This demonstrates
only the first local planning segment, not reaching/passing the obstacle or goal.
Evidence: diagnostics/sando_lidar_longer_timing/summary.json.

## Read-only timed bridge prototype added

sando_bridge_core.py: cubic evaluation, exact interior norm-extremum checks
for speed and acceleration, jerk/altitude limits, segment and replan continuity,
all derivative ENU->NED conversion, yaw conversion, and fail-closed preview
watchdogs for stale state/replans, expiry, timer stalls, wall-clock jumps, and
excess reference distance from state. Six targeted unit tests pass.

sando_to_px4_dry_run.py: 20 Hz evaluation, reads existing state and trajectories
plus PX4 status/landed telemetry, writes a diagnostic report after 15 seconds.
No ROS publishers are created (also checked via AST inspection).
Run `bash diagnostics/check_live_bridge.sh` on the host. Output:
diagnostics/live_bridge_preview.json. Live behavior remains pending this run.

This prototype has no flight-control publishing or implemented flight fallback.
PX4 timestamp synchronization, acquisition-time state alignment, live LiDAR on
X500, map persistence/unknown space, and tracking behavior remain unvalidated.
Working preview/config files were preserved; all new code is additive.

## Live dry-run result: reference/state alignment blocked

15.09 seconds observed; timer maximum interval 0.05083 s (20 Hz stable).
Four own-agent trajectories rejected: initial setpoint >0.75 m from measured
state. No valid previews were emitted. Guard thresholds were not relaxed.
PX4 status fresh: armed=true, nav_state=4 (Hold/auto loiter), failsafe=false,
landed=false. No mode or flight commands were issued by the dry-run.

The current planner was launched with use_state_update=false; source confirms
state callbacks then initialize the planner state once and ignore later measured
state updates. This is a possible contributor to the mismatch, not a confirmed
complete cause. Exact measured/reference positions were not in this first
report. Added alignment positions, distance, state age, trajectory age, and
remaining duration to the next read-only report to distinguish causes.

Remaining work: diagnose state/reference alignment; attach live LiDAR to X500
and validate moving-frame/origin/time handling; complete flight publishing and
fallbacks with verified PX4 timestamps; progressive approved flight tests.

## Alignment diagnostic prepared

Source inspection confirms use_state_update=false initializes state once and
uses getState(A) on each replan; later measured state messages are ignored.
The source calls this branch global-planner benchmarking. The model's DynTraj
pos field reports that internal planner state, making direct comparison possible.

Run `bash diagnostics/diagnose_alignment.sh` on ordinary WSL. It captures active
parameters/connections and 15 seconds of read-only observations, including
receipt-nearest raw PX4/state-bridge positions, planner-reported state, trajectory
start/current position, ages, and polynomial snapshots (up to ten). The offline
classifier distinguishes evidence of frozen-state initialization from reference
timing and state-conversion disagreement. It does not claim acquisition-time
synchronization or alter any running parameters. Previous evidence is backed up.
Imports, no-publish AST checks, and classifier branch checks pass locally.

## Alignment measurements identify delayed polynomial start (not state offset)

Three live samples: polynomial start age 4.414–4.499 seconds at arrival;
header age only 1.7–2.7 ms; evaluated position 1.86–1.93 m from actual drone.
Planner-reported internal position differs from measured state only 1.8–2.7 cm.
Receipt-nearest PX4 conversion and bridge state positions agree exactly in the
reported sample (0.345 ms receipt separation). This rules out a large frozen
initial-state position offset as the cause in these observations. It does not
prove acquisition-time synchronization or global-origin alignment.

Do not rebase polynomial start timestamps to hide the delay: the time origin
belongs to the planned trajectory. Current guard rejection is appropriate.

Prepared diagnostics/test_shadow_alignment.sh: a separate /SH01 planner reads
current /NX01/state and /NX01/sensor_point_cloud, copies the existing goal from
own-agent DynTraj metadata, and publishes own trajectories only on
/sando_shadow/trajs. The original planner and working YAML remain unchanged.
The shadow uses longer time factors 3.0–4.0 and a generated config; all dynamic
limits remain unchanged. A read-only 20 Hz observer checks 15 seconds of output.
No PX4 command publisher exists in the test scripts. Current live map remains
synthetic; this test only isolates trajectory generation delay from state issues.
Syntax/import/remap checks and the six core unit tests pass locally.
Host runtime results pending: diagnostics/shadow_alignment/summary.json.

## Live-input shadow preview PASS

144 trajectories accepted; 291 valid 20 Hz evaluations; zero trajectory
rejections. Only startup waiting (nine ticks) blocked preview. Timer maximum
interval 0.05273 s. Final reference/state difference 0.0082 m. First-ten-sample
summary maximum start age 0.487 s is not a whole-run maximum: final sample age
was 0.751 s. This is still substantially below the original ~4.5 s delay, but
latency must be watched continuously. No flight-control messages were published.
The map source in this test remains the synthetic tree.

Prepared additive world hacktx_forest_x500_sando.sdf with LiDAR variant included
as x500_0; full resolved SDF validation passes. PX4 source confirms attaching to
an existing model via PX4_GZ_MODEL_NAME instead of spawning. Proposed landing
and model-switch procedure is documented in X500_LIDAR_SWITCH.md. User approval
is required for the landing mode change; none has been issued. Read-only
check_grounded.sh verifies fresh landed+disarmed telemetry before any restart.

## Landing approved and verified

User explicitly approved controlled simulated landing. User executed commander
land in the PX4 console. Ground check passed: DISARMED (1), landed=true,
fresh telemetry stable for three seconds; observation duration 4.05 s.
Evidence: diagnostics/grounded_status.json. The check itself sent no commands.

Prepared scripts/start_lidar_forest.sh and scripts/start_px4_lidar.sh for use
after stopping only the old PX4/Gazebo terminals. They use system Gazebo paths,
workspace model resources, and PX4_GZ_MODEL_NAME=x500_0 to attach rather than
spawn. No arm/takeoff/mode/setpoint commands are present. They refuse a second
PX4 process; the forest script also checks for a running default-world server.
Shell syntax checks pass; sourced ROS + system environment resolves Gazebo8.15.
Prepared read-only diagnostics/capture_x500_lidar.sh for new sensor readings,
Gazebo poses, and PX4 status after restart. Model switch not yet runtime tested.

## Actual X500-mounted LiDAR runtime PASS on ground

After user restarted using the prepared launch scripts, both scan and packed
point-cloud publishers are active. PX4 reports landed=true, armed=false.
Three grounded scans each contain 11,106 valid returns; all match copied forest
geometry within 0.10 m. Median surface error ~0.00269 m; five trees detected.
Evidence: diagnostics/x500_lidar/geometry_summary.json.

Gazebo model origin is z=-0.0130008, base_link is +0.24 relative to model,
LiDAR link is +0.54 relative to model (=+0.30 above base), so actual sensor world
z=0.526999 m. Applying only +0.30 to model origin would be wrong. LaserScan's
worldPose remains identity/zero and is not used for transforms. Pose snapshots
are identical but follow scans by ~0.1 s; this proves only stationary geometry.

Original SANDO is still running frozen at its pre-restart airborne state;
trajectories remain rejected by preview while current PX4 state is on ground.
No control publisher has been enabled.

Added perception_geometry.py with tested rigid transform chains, quaternion
interpolation, pose history, and rejection of time resets/gaps/extrapolation.
Nine total unit tests pass. Prepared test_x500_sensor_timing.sh to bridge only
points/poses/clock into preview topics and capture 15 seconds of these alongside
raw PX4 odometry and timesync telemetry. It does not publish to SANDO or PX4.
Host run pending; outputs diagnostics/sensor_timing/. This is needed before
choosing a supported acquisition-time/world-to-PX4-local transform pipeline.

## Timing recorder serialization failure corrected

Bridge successfully created points, pose, and clock bridges; twelve point-cloud
files were saved, but end-of-run metadata failed to serialize NumPy float32
values from PX4's fixed arrays. No timing/frame conclusion can be drawn from
that incomplete run. Failure reproduced locally using actual VehicleOdometry
message type. Recorder now converts array scalars explicitly, handles NumPy
fallback values, atomically writes JSON, and records timestamps/frame directly
inside each NPZ. Callback serialization and atomic-save checks pass. Shell
runner now preserves recorder stdout/stderr. Failed-run evidence backed up under
diagnostics/sensor_timing/before_numpy_fix. Rerun the same host capture command.
No simulation, planner, or flight-control settings were changed.

## Direct named-pose perception preview implemented

ROS Pose_V->TFMessage conversion produced empty parent/child frame IDs because
scene poses lack TF header metadata. Installed Gazebo Python transport13 and
msgs10 bindings are available, so the new preview reads named Pose_V directly.
No Gazebo plugin or working model changes were needed.

PX4 generated serializers confirm timestamp_sample adds DDS time_offset while
TimesyncStatus.estimated_offset is serialized unchanged with opposite sign.
Mapped simulation sample time is therefore (timestamp_sample+estimated_offset)
/1e6 for this configured build. Existing capture comparison to receipt-nearest
sim clock gives differences -55..33 ms (314 samples); that comparison includes
receipt skew and is not itself precise acquisition-time validation.

x500_live_perception_preview.py now buffers/interpolates named simulation poses,
checks the actual 0.30 m mount, maps PX4 odometry sample times using fresh DDS
sync, calibrates world->PX4 ENU from 50 grounded/disarmed pose pairs, and freezes
that calibration. Estimator reset, timing reset, changed mount/origin, stale
odometry/clouds, and missing pose brackets prevent preview publication.
Each accepted cloud is checked against known forest geometry, filtered for self
returns/ground, voxel-downsampled, and published only to
/sando_perception/points_map_preview (px4_enu_preview frame, simulation stamp).
No SANDO inputs or PX4 control topics are published. Partial observations are
not considered proof that unobserved space is free.

Twelve core/geometry tests pass. An offline integration replay using the real
saved cloud and actual named pose protobufs passes calibration, geometry,
preview generation, and stale-odometry suppression. Binding/message imports and
sole-preview-publisher checks pass. Host runtime remains pending:
`bash diagnostics/test_live_perception.sh` (20 s). Outputs live_perception/.

## Expedited PX4-controlled demo requested

User explicitly chose PX4-controlled flight over a Gazebo-kinematic drone.
Prepared a separate quick_px4_demo/ path without removing previous work.
It uses exact static forest geometry and known moving-object trajectories rather
than sensor perception or SANDO outputs. Two red/blue cylinders are inserted
into the existing Gazebo instance and moved via pose services; only those named
cylinders can be targeted by the helper. The X500 remains controlled by PX4.

Default run is preview: no PX4 publishers exist unless --fly is selected.
Flight implementation includes Offboard warmup, command/arming confirmation,
2 m position takeoff, velocity navigation to world XY(12,0), goal hold, landing,
and state/mode/time/alignment/clearance checks. Landing/failure freezes movers;
unexpected flight-mode changes yield control to PX4/pilot. No flight commands
have been run or approved for this new demo yet. Earlier approval was landing
only. The next host step is the default 20-second preview.

A* uses conservative full swept capsules for moving objects to avoid reactive
side-switching traps; local rollouts still check actual predicted motion and
braking. Offline simplified flight with tracking lag reaches the goal at two
obstacle phases while retaining >0.20 m measured modeled body clearance.
All 15 current unit tests pass. ROS message/import compatibility and obstacle
service targeting checks pass. Actual host preview and PX4 flight are untested.
Documentation: quick_px4_demo/README.md. Launch: scripts/run_quick_px4_demo.sh.

## Quick demo first host preview: healthy telemetry, RPC failure

Preview aborted on creating quick_demo_crossing_1, with odometry/Gazebo/status/
DDS/landed telemetry fresh (3–27 ms). Drone remains landed and disarmed. Gazebo's
server log confirms the first cylinder was actually created at 00:43:11.764;
Python's synchronous request nevertheless returned failure. No flight commands
were published. The precise Python transport response failure is not established.

Replaced obstacle RPC with a separately compiled C++ Gazebo transport helper,
communicating over stdin/stdout with the Python worker. It can only create/move
the two named obstacle cylinders; it never addresses x500_0. This also isolates
blocking native requests from Python control-loop scheduling. Creation checks
actual scene names if an async create acknowledgement fails, so prior cylinders
can be reused. Compilation against installed transport13/msgs10 succeeds; local
planner/core tests remain passing. Host retry is still the same preview command.

## Native RPC works; landed-message startup race corrected

Second host preview creates both moving obstacles; native RPC log has no errors.
It aborts at 1.456 s with armed=false and landed=null (not yet received).
This was a controller startup bug: missing landed telemetry was treated as a
confirmed non-landed state. It now waits for fresh landed/disarmed confirmation
(up to the existing startup deadline), while still rejecting definite armed or
airborne state. Added regression tests for missing, stale, ready, armed, and
airborne cases. No safety thresholds were relaxed or flight commands issued.

## Quick PX4 demo host preview PASS

Latest result: PASS, phase PREVIEW, failure=null. Both moving obstacles created,
planner ready, zero blocked planning cycles; timer maximum interval 0.05274 s.
Telemetry remains fresh; landed=true and armed=false. Ground frame calibration
ready at 2.858 s. No PX4 command publishers or flight commands were used.
Evidence: diagnostics/quick_px4_demo/preview_result.json.

Next proposed action requires explicit user approval: --fly mode would request
Offboard and arm, rise 2 m above initial altitude, navigate to world XY(12,0)
around static trees and the moving-object envelopes, then request landing.
Actual PX4 flight/tracking/fallback behavior has not yet been tested. This is
simulator-ground-truth autonomy, not the original SANDO/LiDAR flight pipeline.

## Autonomous demo flight explicitly approved

User: "yes you have my approval now do it" after reviewing Offboard/arming,
2 m ascent, navigation around trees/moving objects to world XY(12,0), and landing.
Approval covers this simulated quick demo. Host execution still requires the
normal WSL terminal because this session cannot reach live simulator/DDS.
Launcher logging now uses tee -i so Ctrl+C does not close the output pipe before
Python's landing handler can finish. Shell syntax check passes. No flight
command has been sent by this sandbox session; awaiting host launch/result.

## First approved PX4 demo flight stopped and landed successfully

Flight log confirms Offboard accepted (cmd176/result0), arming accepted
(cmd400/result0), takeoff completed, navigation began at 21.558 s, and fixed
position-alignment guard triggered at 25.308 s. NAV_LAND accepted (cmd21/result0),
then landed=true and armed=false at 33.958 s. Goal not reached. Control timer
maximum 0.05175 s; modeled measured body clearance >=2.233 m. The controller
continued through landing; a separate GUI freeze is not established by this log.
Original flight report/console were backed up as first_flight_*.

Found code error in demo frame calibration: it inferred position-axis rotation
from estimated attitude vs Gazebo attitude. Installed PX4 GZBridge.cpp explicitly
converts position ENU->NED by swap/sign; zero-heading world position axes match
geographic ENU, with local-origin translation. Estimator heading bias must not
be interpreted as a rotation of those position axes. Demo calibration now
estimates translation only; existing 0.25 m divergence threshold remains intact.
Regression test verifies attitude bias does not rotate geographic positions.
First flight lacked alignment-vector samples, so this is a code correction and
plausible cause, not yet a verified runtime resolution. New flight_alignment.json
and per-second console progress capture the required moving-state evidence.
No new flight command was issued by the sandbox. Same approved demo retry is
pending normal WSL host execution. Previous SANDO/LiDAR work remains unchanged.

## QGroundControl disconnection blocks flight retry

User reports QGC disconnected; autonomous flight retry is on hold. Found existing
Linux AppImage at /home/harneksab/QGroundControl/QGroundControl.AppImage and Linux
QGC configuration/telemetry directory. Actual active Linux vs Windows QGC process
and UDP bindings are not visible from this session's isolated process/network
view. PX4's installed startup source uses local UDP18570 for GCS, default remote
14550, with optional PX4_NET_INTERFACE broadcasting. No network parameters changed.

Prepared read-only diagnostics/check_qgc.sh: uses installed px4-mavlink and
px4-commander POSIX client executables to query the running PX4 instance, plus
host QGC/PX4 processes, UDP sockets, routes and optional Windows task list.
No mode, arm, land, parameter-set or flight commands are issued. Host execution
pending; result diagnostics/qgc/status.txt. This avoids manually copying PX4
console output and distinguishes missing MAVLink, missing QGC, and routing issues.

## QGC diagnostics: local Linux QGC, missing MAVLink link

Host report shows Linux QGC AppImage and PX4 running, Windows QGC absent.
Commander confirms disarmed/no failsafe. MAVLink status was empty and no MAVLink
UDP sockets appeared. Installed Mavlink::get_status_all_instances prints nothing
when there are no instances; the first script did not retain its exit code, so
a timeout cannot yet be excluded. Prepared scripts/reconnect_qgc.sh to distinguish
that case, refuse duplicate startup on a timeout, and start only a missing local
GCS UDP18570->127.0.0.1:14550 link. It enables MAVLink boot_complete and captures
after-status/sockets. It requires disarmed state and issues no flight commands.
If QGC still has no UDP14550 listener, script prints explicit Comm Links steps.
No changes to PX4 source, parameters, flight modes, or QGC configuration files.
Host run pending; autonomous flight retry remains on hold.

## QGC repair follow-up: socket presence confirmed, explicit loopback link prepared

Current host sockets confirm PX4 UDP18570 and Linux QGC UDP14550, plus other
PX4 MAVLink ports. Thus the earlier "no MAVLink sockets" inference was not
reliable: first diagnostic used rg, unavailable in the user's host shell.
Replaced that dependency in the diagnostic too. Empty px4-mavlink status captured
via POSIX client is also not evidence of no instances: boot_complete logs active
localhost links and sockets prove they exist. Direct heartbeat state is needed.

Updated reconnect_qgc.sh to add a bounded, explicit spare telemetry link on
UDP18571 -> 127.0.0.1:14550, leaving existing links unchanged, then observe
VehicleStatus.gcs_connection_lost for eight seconds. Existing spare link is
reused. No flight-mode/arm commands or parameter changes occur. The quick demo
now refuses flight startup if a GCS heartbeat is missing, and requests landing
if it loses that heartbeat while it owns Offboard. Source/message compatibility
and syntax checks pass. Host reconnect/heartbeat verification pending.

## Second flight: navigation works, wall-clock synchronization guard aborts

Latest flight takes off, enters NAVIGATE at 14.610 s, reaches world XY
(2.540,-4.290), then aborts at 20.887 s on "DDS timestamp synchronization stale".
Landing is accepted and completes landed/disarmed at 29.713 s. Position frame
fix works in this flight: maximum recorded alignment residual 0.1183 m, below
unchanged 0.25 m limit. Measured modeled body clearance >=0.7177 m; zero blocked
planning cycles; control timer maximum 0.05128 s. Goal still not reached.

The old synchronization guard combined missing sync messages with an absolute
OS wall-clock vs DDS-adjusted odometry timestamp comparison. This is invalid
for simulation clocks which are bridged to OS time through an estimated offset;
precise failure operands were not recorded, so the combined guard does not
identify which operand fired. Corrected command generation to use the timestamp
from fresh PX4 DDS feedback, preserving the same adjusted clock domain on round
trip. Installed serializers confirm outgoing timestamps add time_offset and
incoming timestamps subtract it. No timestamp=0 arrival-time bypass is used.

Removed the OS wall-clock equality requirement, retaining 0.3 s state freshness,
2 s DDS sync-receipt freshness, mode/GCS/reset/clearance checks and PX4's own
message timestamp handling. Setpoint/heartbeat publisher history reduced to one
sample. Added explicit stale-feedback timestamp regression test and diagnostic
wall/feedback-clock deltas plus sync age. Flight landing handler does not stream
new setpoints from stale odometry. User-requested full route remains the target;
no success claimed until goal arrival. Earlier flight evidence backed up.

## Successful baseline flight and optional slalom prepared

Latest baseline flight PASS: goal reached and landed/disarmed, no failure;
minimum modeled measured body clearance 0.5972 m, timer max 0.05157 s. Flight
completes at 44.349 s. Full working quick demo source/binary/launcher and actual
success logs backed up in backups/px4_demo_success_before_slalom/.

User requested a fresh run with visible weaving. Added --slalom profile with
three 0.65 m radius colored cylinders moving +/-0.35 m about staggered centers
(2.8,0), (6.5,2.7), (10.5,-2.2). Route checkpoints:
(0,0),(2.8,2.8),(6.4,-0.4),(11.2,1.2),(13,0). From the previous finish, it first
flies back to the start. No vehicle/world reset or drone pose command is needed.
Checkpoint transitions are logged and all must be reached for mission success.
Global grid is 0.20 m for this profile; finer resolution resolves the real
inflated gaps, without reducing body or safety margins. Global paths remain
stable per leg; moving-obstacle prediction and velocity/braking validation
continue every cycle. This is explicitly waypoint-guided ground-truth slalom,
not a new demonstration of onboard sensing or SANDO control.

Native helper supports the profile and third cylinder; returning to baseline
removes only that extra named demo cylinder. Runtime spawning waits for fresh
landed+disarmed status. Python/native motion parameters are checked for equality.
Twenty tests pass, including full return-home/slalom completion with tracking
lag at phases 0,12,25 s, zero blocked cycles and modeled body clearance >=0.4145 m.
No slalom host flight has been executed by this session. Same PX4 flight interface
and safety watchdogs retained; default successful baseline profile preserved.
