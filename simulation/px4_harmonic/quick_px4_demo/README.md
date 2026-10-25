# Separate PX4 moving-obstacle demonstration

This preserves all SANDO, LiDAR, world, and bridge work. It is a faster alternate
**ground-truth simulation** demonstration, not a completed SANDO/LiDAR integration.
PX4 controls the real simulated X500 via Offboard; the drone is never teleported.
Only the two named red/blue demo obstacle cylinders are moved through Gazebo pose
services. Tree geometry comes from the existing forest. Known sinusoidal obstacle
motion is used for prediction.

The existing LiDAR forest Gazebo world, attached PX4 SITL, and DDS agent must be
running, with the drone landed and disarmed. No simulator restart is needed.

## Preview (no flight-control publishers)

```bash
bash ~/hacktx/scripts/run_quick_px4_demo.sh
```

This creates and moves two cylinders, checks live state and frame calibration,
and exercises the route/prediction planner for 20 seconds. It does not arm,
change flight mode, or send PX4 setpoints. Output is saved under
`diagnostics/quick_px4_demo/preview_result.json`.

## Flight — requires explicit user approval first

The separate `--fly` mode requests Offboard after a two-second setpoint warmup,
requests arming, rises 2 m above its initial altitude, navigates to world XY
(12,0), holds briefly, and requests landing. It does not forcibly disarm in air.
It checks command acknowledgements, current flight mode, state freshness,
DDS timestamp freshness, estimator resets, control timing, position alignment,
and measured obstacle separation. A failed plan freezes obstacle motion and
requests landing when the demo owns the Offboard flight. If the pilot/PX4
switches modes, it stops issuing flight commands and yields.

Planning uses conservative A* around each mover's complete swept capsule, then
acceleration-limited local rollouts checked against predicted moving positions
and a braking tail. Command speed is at most 0.65 m/s; command acceleration is
limited to 0.45 m/s². Drone collision radius is 0.65 m with extra planning margin.
These commanded limits are not proof of actual PX4 tracking limits. Actual flight,
actuator behavior, command acceptance and failure handling require host tests.

Offline tests reach the destination with moving obstacles and a simplified
tracking lag at two different obstacle phases. They do not validate PX4 flight.
The full-stack host preview and first flight remain pending.

The original SANDO may stay running because this demo does not subscribe to its
trajectories. Any other publisher on PX4's trajectory-setpoint topic blocks flight
startup. The launcher prevents concurrent copies of this demo. Original startup
scripts and files are unchanged. Demo obstacles freeze when the process stops;
the original forest can be restored by restarting its original world after
verified landing/disarming.

## Slalom profile

The original successful two-obstacle profile remains the default. Before this
change, its complete source, helper binary, launcher and successful flight logs
were copied to `backups/px4_demo_success_before_slalom/`.

The optional `--slalom` profile uses three colored moving cylinders and a finer
0.20 m global planning grid. Its ordered checkpoints are world XY:

`(0,0) -> (2.8,2.8) -> (6.4,-0.4) -> (11.2,1.2) -> (13,0)`.

If the drone last landed at the old finish, it first takes off and flies back
to `(0,0)`, then performs the outbound weave without restarting the simulator.
There is no drone teleportation. The new checkpoints shape a visible north /
south / north weave through spaces between the moving cylinders and existing
trees. They are route constraints, not proof of spontaneous obstacle detection:
this remains the exact-ground-truth demonstration, with predicted local collision
checks and braking tails used to select safe velocities.

```bash
# No flight commands; preview the new obstacle layout and first route.
bash ~/hacktx/scripts/run_quick_px4_demo.sh --slalom

# Flight mode: use only with the user's flight authorization.
bash ~/hacktx/scripts/run_quick_px4_demo.sh --fly --slalom
```

A successful run records all five visited checkpoints before landing. Stable
conservative global paths are retained for each slalom leg, since obstacle
movement remains within the known swept bounds; local velocity planning still
updates against their time-dependent positions. Profiles share the same flight
watchdogs and emergency landing behavior. Native helper configuration is tested
against the Python planning model to prevent obstacle/model mismatch.

Twenty local tests pass, including all checkpoints in order with a simplified
tracking lag at three different obstacle phases; modeled body clearance remains
above 0.41 m. The slalom PX4 host flight itself has not yet been run.

## Challenge profile: twelve moving obstacles, return course

`--challenge` keeps the three central slalom cylinders and adds nine moving
cylinders on the north/south boundaries and east end of the course. All twelve
have modeled collisions and predicted sinusoidal motion. The flight visits nine
checkpoints: the five slalom checkpoints followed by `(11.2,1.2)`, `(6.4,-0.4)`,
`(2.8,2.8)`, `(0,0)`. This tests both travel directions and repeated turns without
increasing speed or reducing clearance margins. Extra traffic surrounds the
corridor; it does not imply twelve close encounters or unpredictable motion.

```bash
# Start here, landed/disarmed: moves objects, never publishes flight commands.
bash ~/hacktx/scripts/run_quick_px4_demo.sh --challenge
# After inspecting the layout, using the existing flight-test authorization:
bash ~/hacktx/scripts/run_quick_px4_demo.sh --fly --challenge
```

The baseline and slalom profiles remain available. Selecting either removes
only surplus named demo cylinders left by the challenge. No drone or forest
model is removed. The pre-change implementation is in
`backups/before_challenge/quick_px4_demo/`.

Offline testing with simplified tracking lag at three starting phases completed
all nine checkpoints in 84–85 seconds of modeled navigation, with minimum modeled
body clearance above 0.40 m. Actual PX4 flight and simulator timing under the
larger obstacle load remain untested. Allow roughly two minutes per host flight;
watchdogs still request landing if planning or telemetry becomes unhealthy.
This remains known-geometry simulation testing, not validation of live perception,
SANDO integration, unknown trajectories, or real-world safety.

## Browser manual override and physical contact test

The optional `--manual-control` starts the existing leased WebSocket command
server on loopback port 8766, inside the sole PX4 controller. It is a simulation
feature: holding Forward/Back/Left/Right commands 0.8 m/s body-relative horizontal
velocity (heading captured on press), with altitude held at the takeoff target.
Obstacle avoidance is deliberately bypassed during the hold; collision is allowed.
The browser renews a 500 ms lease every 100 ms. Release, disconnect, loss of focus,
or expiry withdraws the override. Fresh PX4/Gazebo feedback and active Offboard
navigation are required. Arming, takeoff, and landing cannot be overridden.

Release brakes and rebuilds a route from the current position to the current
checkpoint, retaining mission progress. The normal collision checks resume. If no
fresh safe route is available within three seconds, the controller requests
landing. Release cannot guarantee recovery from arbitrary proximity, excessive
tracking error, or an impact. Existing mode, GCS, timing, estimator and telemetry
watchdogs remain in effect. Normal course speed is unchanged.

The old forest includes `collide_without_contact` flags. For collision tests use
the separate `hacktx_forest_solid.sdf`, where those flags are false and 41 tree
contact sensors plus the Contact system are enabled. Demo cylinders also have
contact sensors. No drone model or original forest is modified. Restarting into
this world removes old demo cylinders; the next run creates the updated versions.
A `/demo/contacts` report involving x500_0 during navigation latches a collision
failure and requests landing, without forcibly disarming in air. It does not fake
a tumble or a crash animation: physical response depends on Gazebo and PX4.
Collision is established by contact messages, not center-distance estimates.

After the current run is finished and the drone is confirmed landed/disarmed:

1. Stop PX4 in its own terminal, then the old Gazebo server in its own terminal.
2. Gazebo terminal: `bash ~/hacktx/scripts/start_solid_forest.sh`
3. PX4 terminal: `bash ~/hacktx/scripts/start_px4_lidar.sh`
4. Dashboard terminal: `bash ~/hacktx/scripts/start_live_dashboard.sh`
5. Demo terminal: `bash ~/hacktx/scripts/run_quick_px4_demo.sh --fly --challenge --manual-control`
6. Open `http://localhost:8765/?source=live&control=ws://localhost:8766/control`.

The controller command server must be running before opening that URL; if opened
earlier, reload after starting it. Buttons enable during navigation. Pause remains
unavailable. Keep QGC connected. `commander land` at the PX4 `pxh>` prompt remains
the pilot override. Do not stop PX4 while airborne.

The dashboard's persistent collision indication reads controller event snapshots
from diagnostics/quick_px4_demo/manual_state.json. They survive command-server
shutdown and clear when a new manual-enabled run starts. Actual moving geometry
and the drone pose still come from live Gazebo; no collision outcome is fabricated.
The live view shows current geometry; accumulated history may retain old mover
positions. Tests validate lease ownership, ordering, expiry, release, directions,
collision latching, and world XML. Physical collision response, contact topic
publication, PX4 execution, and release/rejoin remain host integration tests.

## Dashboard flight lifecycle

`bash ~/hacktx/scripts/start_dashboard_controller.sh` enables the opt-in persistent
standby mode. It waits for the dashboard Start button before emitting flight
setpoints or requesting modes/arming. Return Home follows the existing checked
planner back to the calibrated takeoff position and lands. Successful completed
runs can be started again from the dashboard; failed runs require restart after
inspection. See frontend_adapter/README.md for setup and validation limits.
