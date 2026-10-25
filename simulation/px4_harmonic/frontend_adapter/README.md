# Live dashboard for the existing PX4 forest demo

Run in a normal WSL terminal while the existing Gazebo world is running:

    bash ~/hacktx/scripts/start_live_dashboard.sh

Open http://localhost:8765/?source=live in the Windows browser. Leave this terminal
running. Run flight commands in a different terminal. No ROS build is needed.
The launcher installs websockets 13.1 in a separate system-site-packages venv,
leaving ROS and the flight controller untouched.

This is a read-only ground-truth visualization, not a sensor map. Static cylinder
surfaces come from gazebo_worlds/hacktx_forest.sdf. Moving demo cylinders and the
X500 model origin come from live /world/default/pose/info in Gazebo world axes.
The model origin is not the PX4 local origin or drone base_link. Map replacement
is capped at 5 Hz, WebSocket output at 10 Hz. No updates are generated when pose
data stops advancing; freshness continues aging. A simulation clock reset requires
a bridge restart. Unknown actors are omitted; this adapter is specific to the
existing demo world. Historical accumulated points may retain old mover locations;
use Live for the current scene.

This adapter creates no ROS publishers, sends no Gazebo service requests, and does
not start or stop flights. Steering is unavailable. Camera, battery, flight time,
and velocity are not supplied. The visible trail is received model pose history.

Local syntax and dashboard tests can be checked here; live Gazebo/WebSocket
connectivity must be verified on the host. Errors are logged to
 diagnostics/frontend/live_dashboard.log.

Manual-control connection update: the dashboard served on port 8765 now attempts
its loopback command server automatically at ws://<dashboard-host>:8766/control.
An explicit `control=` query parameter still overrides it. A missing server or
closed connection retries every 1.5 seconds; reconnect never revives a held press.
Other static preview ports retain explicit control opt-in. After editing, reload
the browser to load the new JavaScript. Each start/release/rejection is logged as
`BROWSER CONTROL:` in the controller console. `MANUAL:` confirms actual takeover;
`MANUAL RELEASED:` confirms braking/rejoin. Connection fixes are tested locally;
actual browser-to-PX4 execution still requires a live host test.

## Pinned destinations and projected route

After restarting the dashboard adapter and starting the updated controller with
`--fly --challenge --manual-control`, hard-refresh the browser. In Live, choose
**Place pin**, click the local top view, then **Go to pin** during navigation.
The selected XY stays at the normal flight altitude. Acceptance replaces the
remaining challenge checkpoints with that destination; arrival holds and lands.
Pins do not arm or take off. The controller rejects blocked/out-of-bounds goals,
stale command epochs, and goals submitted while a steering lease is held. Manual
steering and the original course remain available until a pin is accepted.

The dashed blue line is the global path to the active checkpoint or pin. The
orange line is the planner's selected four-second candidate rollout, including its
braking tail, refreshed with moving-obstacle predictions. It is distinct from the
lime flown trail and is not proof of PX4 tracking or safety. The global path uses
full swept obstacle envelopes and can remain unchanged while the local prediction
changes. No path animation is fabricated. Predicted paths disappear during manual
override and when controller updates are stale. A pin changes the route; it does
not change obstacle-avoidance margins or permit collisions in autonomous mode.

Controller state is exported atomically at up to 5 Hz to
`diagnostics/quick_px4_demo/live_plan.json`. The adapter accepts fresh state in
Gazebo world coordinates. Flight duration is controller elapsed time from takeoff;
a stopped controller's last received duration is labeled. Battery comes from
`/fmu/out/battery_status_v1` when fresh/connected, labeled PX4 SIM. Otherwise 100%
DEMO is a placeholder. Before a controller run, duration is a labeled 00:00
placeholder. These fields are never presented as hardware measurements.

The operations grid is sized to desktop viewports at least 980 x 600 CSS pixels.
Optional flight preferences open in a popover; small phones keep a scrolling
fallback to preserve usable controls. Browser zoom changes the CSS viewport size.
Unit/DOM tests do not establish actual layout on every screen or live flight.

Apply updates without restarting the simulator: stop only the dashboard adapter
with Ctrl+C in its own terminal, rerun `scripts/start_live_dashboard.sh`, then
Ctrl+Shift+R in the browser. Start the updated flight controller only after the
previous flight has landed/disarmed and QGC is connected. Pinned destinations and
manual control still need an end-to-end host test.

### Pin interaction and fixed battery update

Battery is now a fixed 97% display per user request, captioned BATTERY · FIXED;
this does not alter PX4's actual battery state or stored telemetry. After choosing
Place pin, either the top view or main perspective map accepts a click. The
perspective click is projected onto the drone-height plane; it is not surface
picking. Pin mode uses a crosshair, suppresses camera panning, tolerates small
pointer jitter, and captures the camera transform at pointer-down so a moving
follow camera cannot shift the chosen point. Go to pin becomes available once
the running flight controller has fresh telemetry, calibration and QGC heartbeat,
including preparation/takeoff phases. Steering stays restricted to navigation.
Pins do not themselves arm or take off. Clicks above the camera horizon and
backend-infeasible destinations return visible rejection messages.

The local regression test executes actual dashboard pointer handlers in both
views, sends navigate over the adapter's socket, and checks the acceptance UI.
Live browser/PX4 validation still needs a host run. Apply with a hard refresh and
an updated demo-controller run after verified landing/disarming.

### Destination delivery fix (supersedes the two-click instructions above)

After Place pin, clicking either map now submits the destination immediately.
There is no separate Go confirmation. If the running flight controller is still
preparing, the UI labels the pin queued and sends it once navigation readiness is
reported. Retry pin is available for an explicit retry. Accepted/rejected status
comes from the controller, rather than treating the locally drawn marker as an
accepted goal. Changing sources clears queued pins. Accepted pin destinations
reset the per-destination navigation timeout; the overall flight timeout remains.

The dashboard server now relays `/control` on its existing port 8765 to the
controller's loopback 8766 endpoint, retaining the same command schema, leases,
origin checks and PX4 command owner. No second browser-facing port is required.
Use http://localhost:8765/?source=live&control=ws://localhost:8765/control .
The relay must be loaded by restarting only the dashboard adapter. Hard-refresh
the browser and run the updated controller after landing/disarming.

Diagnostics: dashboard log prints CONTROL LINK / CONTROL REQUEST; controller log
prints BROWSER CONTROL / PIN DESTINATION. A drawn but rejected or queued pin is
not evidence that PX4 received a new destination. Local tests cover both real UI
pointer handlers, queued readiness, proxy command/ack forwarding, and a planner
run to the new goal instead of the old one. Host browser/PX4 flight remains to be
verified; socket relay tests use mocked sockets rather than host network access.

### Pin reliability and occupied-goal handling

The latest recorded rejected request was `Goal inside inflated obstacle`.
The dashboard now outlines the planner's reserved static circles, full moving
swept capsules, and course boundary at flight height. Exact feasible pins are
unchanged. An infeasible pin may snap to a reachable clear point within 2.5 m;
the controller acknowledges the actual XY and adjustment distance, and the marker
moves to that accepted XY. A point with no nearby reachable alternative, or one
outside X [-2,16], Y [-10,10], is explicitly rejected. This does not reduce
obstacle margins or command the drone into a tree.

Rapid selections are serialized: the latest desired point waits for the current
request's acknowledgement, then uses its updated control epoch. Earlier replies
cannot overwrite the newest marker. Only transient freshness/epoch/held-control
rejections are queued for retry; geometric rejection is final until a new pin.
Checkpoint completion and destination mutation now share the controller lock so
a simultaneous pin cannot accidentally be marked reached by the old checkpoint.

Tests cover rapid pins, adjusted-target acknowledgements, geometric resolution,
and an offline tracking run to the resolved pin. They do not replace actual host
browser/PX4 testing. Apply with a browser hard refresh and the updated controller
on the next landed/disarmed flight run. Fixed battery display stays at 97%.

## Start, Return Home and live speed

Use the new standby launcher instead of the automatically flying demo command:

    bash ~/hacktx/scripts/start_dashboard_controller.sh

It starts the same PX4 controller with `--wait-for-start`, and remains grounded
until the dashboard Start command is accepted. Fresh telemetry, QGC heartbeat,
frame calibration, a valid plan, and landed/disarmed status enable Start.
The button then requests the existing warmup/Offboard/arm/takeoff/course sequence.
After successful landing/disarming the controller remains available for another
Start, recalibrating at that run's actual takeoff position. A failure/collision
requires resolving the issue and restarting the controller; Start is not a
crash-recovery or airborne rearming control. The original auto-flight launcher
continues to work, but its Start button stays unavailable.

Return Home is enabled during takeoff/navigation. It supersedes manual steering
and destinations, routes to the recorded takeoff XY at the normal flight altitude,
and lands on arrival. It uses the existing obstacle-aware route/local planner,
not an unverified straight-line native RTL command. Home is exact: the pin's
nearby-point snapping is disabled. An unavailable safe return route is reported
rather than silently choosing another destination. Initial home is captured from
Gazebo body pose after ground frame calibration.

Controller exports now include fresh PX4 velocity transformed NED -> ENU ->
Gazebo world. The dashboard receives the XYZ m/s vector and displays its 3D norm
in Flight dynamics. Missing/stale PX4 velocity shows a dash rather than a fake
speed. Battery remains the user's fixed 97% display.

Restart the dashboard adapter once to load velocity forwarding, hard-refresh the
browser, and run the standby launcher only after the previous flight is verified
landed/disarmed. Keep Gazebo, PX4, DDS and QGC running. Local tests cover actual
Start/Return Home button handlers, command acknowledgements, standby transition
gating, exact home routing, and velocity export/freshness. Live flight of these
new controls still needs host validation.

## Landing and near-obstacle startup recovery

The recorded stuck location (9.37, 0.49) is now covered by regression tests.
If the normal planner cannot start because of its extra buffer, the worker tries
a separate low-speed recovery: maximum 0.15 m/s, acceleration 0.20 m/s². Recovery
is admitted only with sufficient current body separation and low measured speed.
Its actual issued command and braking tail are checked against every tree and
full moving-obstacle swept capsule. Separation from nearby constraints cannot
reduce during the verified escape; the moving-object timing reserve is retained.
The controller's 0.25 m guard is unchanged. Once conservative clearance reaches
0.70 m, normal planning resumes. Standby still sends no movement commands; Start
must be clicked and the normal takeoff must finish before horizontal recovery.
No model teleportation or obstacle-collision disabling is used.

At nominal mission completion the controller selects a reachable nearby landing
site with at least 0.85 m conservative swept clearance, navigates there if needed,
and position-aligns within 0.12 m while velocity is below 0.12 m/s for 1.5 seconds
before requesting PX4 landing. This replaces the previous immediate brake/land
at the 0.4 m waypoint-arrival threshold. Return Home may choose an adjacent clear
landing site rather than descend into an unsafe takeoff buffer. Adjustments are
logged. Emergency landing/failsafe paths retain their priority and do not promise
this nominal landing clearance.

Apply after verified landing/disarming: stop only the old dashboard controller,
run scripts/start_dashboard_controller.sh again, and hard-refresh the browser.
The existing stuck position can be used to test recovery without resetting the
forest, provided body separation is sufficient. A real collision or an unverified
escape still blocks startup. Prior failure latches clear on controller restart.
Local tests cover the actual standby worker, three-phase lagged escape and onward
navigation, landing-site selection with tracking offsets, rejected unsafe escape,
and the actual final landing hold gate. PX4 recovery/descent behavior still needs
a controlled live Gazebo test.
