# Flight room and operator dashboard

Local, buildless browser UI. No Gazebo, ROS, npm install, CDN, or API key is
needed to inspect the synthetic mission. It is a UI prototype, not a validated
flight-control implementation.

## Simple flight room (default)

The default page is a 32 × 24 m room with one drone, three interior partitions,
a sofa, armchair, coffee table, work desk, work island, and shelving. The center
remains open for flight. Furnishings are fixed collision geometry; no moving
objects are preloaded. Choose **Fly here**, **Static object**,
**Moving object**, or **Remove**, then click directly in the 3D room. Drag to orbit,
shift-drag to pan, and scroll to zoom. **Recenter** fits the whole room.

**Fly here** uses the selected destination height. Build the room and select a
destination, then click **Start**. Before Start, both the drone and moving objects
remain stationary; setting a goal, keyboard shortcuts, and pushing cannot start
them. After Start, choosing a new goal resumes paused flight and the drone holds
on arrival. Reset returns to a stationary room and requires Start again. Pushing without a destination
moves only while held; release holds the current position. Hold the push buttons (or W/A/S/D and Q/E) to override movement;
release returns to the chosen goal. Push leases expire after 500 ms. Blur, hidden
pages, pointer cancellation and key release end a push. Space pauses the world.
Moving objects have a placement orientation: **Left–right (room X)** or
**Up–down (room Y)** across the floor. Choose it before placing each object;
existing objects keep their axis when the placement selector changes or the camera
orbits. Click an existing object to select it, then use **Move X** or **Move Y**
in its selection panel. A static object becomes moving; a moving object changes
direction from its current position without teleporting. The changed scene is
replanned, and the Start gate remains in effect.
Predictions follow that same axis. Moving objects yield to the drone, static objects, and
other moving objects. Clear added objects removes custom objects while preserving the furnished room.
Reset also clears the flight trail and returns the drone to its starting position. Objects are limited to 24 per room session.

The room uses a 0.62 m vehicle radius, enclosing the rendered arms and rotors.
Moving objects are checked against the stationary drone before advancing; drone
and object motion are then checked over the same elapsed interval, including
relative-motion sweeps between sample times. Blocked movement holds and replans;
held pushing uses the same checks. Clearance telemetry measures from this rotor
footprint. `test/collision.test.mjs` covers stationary head-on encounters, crossing
paths, direction changes, pushes and irregular frame timing.

The floor heatmap shows proximity influence around collision geometry using a
Gaussian distance falloff; it is a visualization, not a temperature measurement,
collision probability, or the SANDO optimizer objective. Amber dashes show short
moving-object forecasts. Cyan is the planned route; lime is the actual flown
trail. Telemetry shows speed, body clearance, distance to goal, distance flown,
and plans/arrivals.

This view shares the browser SANDO approximation: inflated A*, bounded cubic
segments and swept checks. It plans horizontally with conservative full-height
obstacle bounds, with vertical transitions to the selected altitude. It does not
run the full ROS/Gurobi/MINVO solver, and segment joins can have acceleration
discontinuities. The existing integration dashboard is at **/dashboard.html**;
**/?source=live** redirects there for the bridge connection.

The default view is implemented in `web/room.mjs` and `web/room.css`; the operator
dashboard remains in `web/dashboard.html` with `web/app.mjs`. Both are bundled
and explicitly allowlisted by the bridge server. `test/room.test.mjs` covers 3D
picking, static/moving object edits, goal altitude, pause, push handoff, keyboard
release on blur, reset, and six simulated minutes of navigation.

## Open locally

From `integration/slam_dunk_bridge` (or `hacktx-transfer/integration/slam_dunk_bridge`
in the parent SANDO checkout):

```sh
python -m http.server 8080 --bind 127.0.0.1 --directory slam_dunk_bridge/web
```

Open **http://127.0.0.1:8080/** for the flight room, or **http://127.0.0.1:8080/dashboard.html** for the operator dashboard. The synthetic mission is visibly labeled. The
Live screen contains the top view, manual controls, speed, margin profile,
main 3D view, drone camera panel, battery, and flight time. History
contains the combined-site recorded map, replay cursor, playback, and details. Orbit,
zoom, recenter, follow, expand, and keyboard controls work locally.

The demo and its in-memory history restart when the page refreshes. History is
bounded and has no disk persistence. An accumulated occupancy sample is not
an authoritative global map: old obstacles can remain in this view, occupied
snapshots do not establish free space, and eviction can remove old geometry.

## Connect the existing bridge

The Python bridge serves the flight room at `/` and the operator dashboard at `/dashboard.html`; the original viewer remains
at `/legacy`. Run the bridge as described in [README.md](README.md), then open
**http://127.0.0.1:8765/?source=live**, or choose Bridge in the source selector.
The static preview server alone has no `/stream` endpoint. A failed live
connection stays visibly disconnected and never falls back to generated data.

The browser consumes protocol v1 map, pose, and status messages from `/stream`.
Map messages replace the current point set, including an empty map that clears
it. Source timestamps, frames, finite coordinates, and payload bounds are
checked before state is accepted. The recorded combined-site sample is separate
from the current map. Connection loss retains old geometry with stale labels.

The existing telemetry bridge is output-only. Optional held steering uses the
new, separately configured `mission_supervisor` sidecar, which owns navigation
destinations and expires steering leases after 500 ms. Hold a direction to
steer; release it to resume autonomous waypoint exploration. Mouse/touch
cancellation, keyboard release, focus loss, hidden pages and connection loss
remove the override. Reconnect requires a fresh press. The center Pause/Resume
button works in the synthetic demo; physical Pause remains unsupported and
disabled until SANDO has a braking/hold service. No browser control publishes
directly to ROS actuator setpoints.

See [SANDO_CONTROL.md](SANDO_CONTROL.md) for the ROS executable/configuration,
camera stream setup, command protocol, readiness gates and verification limits.
The underlying planner braking work remains described in [the control design](../../docs/operator-push-stop-ros-design.md)
in this repository. A stale-data fault does not physically stop the drone.

## Margin slider

The profile uses configured values from the parent `config/sando.yaml`:

| Slider | `inflation_hgp` | `dyn_base_inflation_m` |
| --- | --- | --- |
| 0 — configured baseline | 0.45 m | 0.10 m |
| 100 — more clearance | 0.90 m | 0.30 m |

Intermediate values interpolate linearly. The upper bounds are proposed UI
limits, not experimentally validated operating settings. The slider never
reduces these configured baselines, changes the drone footprint, or disables
unknown-boundary inflation. It does not estimate collision probability.
Increasing inflation can make passages infeasible. These are planner parameter
values, not measured separation distances. In particular, dynamic obstacle
padding can combine the dynamic base with static inflation, so the dynamic base
alone is not the effective dynamic clearance.

SANDO currently reads these settings during initialization. A future backend
must deliberately apply/reinitialize the affected planner state and acknowledge
the active values; setting a ROS parameter alone does not establish that the
running planner changed. Bridge mode therefore labels profiles as proposals.
Synthetic mode applies these values to the browser obstacle planner only. Speed preference is separate from the
clearance profile and is not a replacement for measured velocity.

Battery, flight duration, and velocity extensions are optional pose fields:
`battery` in percent, `flight_seconds` in seconds, and `velocity` as XYZ m/s.
The existing bridge does not publish these fields; they show as unavailable
instead of fabricated measurements. The synthetic source supplies them with
its demo label. The RGB camera panel accepts an external HTTP(S) MJPEG image
stream and shows offline when none is configured. It does not synthesize camera
footage or infer per-frame freshness from image-load events. Camera recording
and dynamic-object tracking are not implemented.

## Verification

From this package directory:

```sh
node test/run_ui_tests.mjs
python -m unittest discover -s test -v
node test/control_contract_smoke.mjs python
```

See `test/UI_TEST_REPORT.md` for the executed coverage and verification limits.
Synthetic/property and DOM/canvas test cases do not validate Gazebo topic wiring,
physical braking, planner feasibility, or hardware behavior.

## Interactive obstacle lab

The synthetic source now runs a furnished indoor obstacle course with three moving
patrols. Choose a destination in **Obstacle lab**, or double-click the top view to
place a goal at the current flight altitude. Cyan dashes show the future route;
lime shows the recorded trail, and amber circles show patrols with inflated bounds.
Held steering uses the same collision checks. Space pauses/resumes when a map has
keyboard focus. Pause freezes the synthetic world. Restart resets the course,
profile, and in-memory mission history. A selected destination pauses on arrival;
resume to continue the autonomous tour. Bridge mode hides the lab.

`web/planner.mjs` borrows the static/dynamic inflation configuration and cubic
position/velocity/acceleration/jerk equations from
`simulation/car/planner/car_sando_planner/config/sando.yaml` and
`src/sando/gurobi_solver.cpp`. A* uses a 0.5 m grid with swept edge checks,
line-of-sight shortening, a 0.24 m vehicle radius, static inflation, and additive
dynamic inflation. Cubic segment durations satisfy analytic speed, acceleration
(2 m/s²), and jerk (4 m/s³) bounds within each segment. The latter two are browser
demo settings. These stop-to-stop segments have acceleration discontinuities at
joins; they are not a globally jerk-bounded SANDO solution.

Moving-obstacle checks sample swept edges at up to 0.08 m / 0.04 s, preview 0.7 s,
and trigger replanning when blocked. Synthetic patrols independently yield at the
vehicle's inflated clearance boundary; this cooperative actor behavior is not a
claim of avoidance against arbitrary real obstacles. Planning is horizontal and
uses conservative full-height occupancy even for furniture. The browser knows
the full scene and does not perform SLAM, MINVO optimization, or the ROS/Gurobi
solve. Changing clearance while already too close to geometry can leave no
feasible route; use Restart to restore the baseline. Live bridge behavior is
unchanged and the browser lab sends no ROS commands.

The obstacle regression suite is included in `node test/run_ui_tests.mjs`: it
checks route edges, inflation, blocked goals, cubic derivative bounds, pause,
and ten simulated minutes of autonomous navigation and patrol encounters.

## Judge-built course and drone POV

Refresh the dashboard after updating the files. **Drone POV** is the default demo
view: a local first-person projection of the current vehicle pose, walls,
furniture, moving patrols, and judge-added obstacles. This is labeled synthetic
camera rendering and is separate from the external RGB stream panel. It uses
no footage, CDN, or extra installation. Full-height occupancy is conservatively
visualized with tall furniture/racks; the geometry is a navigation demo.

To build a challenge:

1. Click **＋ Obstacle**. This opens **Build arena**, a large overhead editor.
2. Choose **Crate** or **Barrier**, then click open space to place it. Orange
   geometry is judge-added; the dashed padding shows static inflation.
3. Choose **◎ Goal** and click a destination, or use the destination dropdown.
4. Switch to **Drone POV** to watch the vehicle navigate the changed scene.
5. Use **− Remove** to remove an added object, or **Clear added obstacles**.

Placement is limited to 24 objects per browser session. Existing geometry,
building bounds, current patrol positions, and the drone/active goal clearance
are protected. An obstacle may close a passage: the planner then reports no
feasible route and holds until the scene or goal changes. The future route is
invalidated on every edit; added objects participate in A*, line-of-sight checks,
manual collision guards, clearance telemetry, map geometry, and POV rendering.
Synthetic patrols yield to added obstacles. Restart clears the edited course.
Bridge mode exposes neither the synthetic camera nor scene editing.

The same SANDO approximation and derivative limitations described above apply.
`test/arena.test.mjs` checks an obstacle blocking the original route, continued
navigation around it, rejected placements, removal, scene isolation, and POV
ray geometry. UI tests cover editing tools, view switching, and source isolation.

## Mission map and replay

Refresh the flight room to load recording support, then use **Mission map &
replay** in the header. It switches to `/#mission-map` in the same browser tab. **Flight room · controls**
and **Back to room controls** return to the existing room without a reload or
reset. The simulation and recorder stay mounted while the replay view is open;
push input is released on switching and cannot be started from replay. The old
`/mission-map.html` URL redirects into this workspace. Only runs made after recording
support is loaded can be reviewed.

The default map accumulates synthetic 360° planar range hits out to 8 m. Each ray
stops at its nearest obstacle; hidden geometry is not marked observed. The known
room boundary is shown as reference. Old hits persist when objects move, so this
is historical evidence rather than a current occupancy grid or a real SLAM map.
**Show full scene reference** adds the simulator's recorded obstacle positions.

The timeline synchronizes drone position, flown trail, planned route, goal,
obstacle state, speed, clearance, distance, and events. Scrub, play, switch between
2D and 3D, or click an event to inspect its time. Follow latest returns to the
newest recorded frame. Map controls are read-only and never control the drone.
Events include Start, goals, obstacle edits, pushes, pauses, replans, arrivals and
avoidance holds. Download mission JSON saves the selected run locally.

The recorder samples motion at up to 2 Hz, with extra frames for events. Identical
stationary frames are skipped. Local storage retains up to 3,600 frames, 600
recent events, 8,000 observed cells, and three prior runs. A 1.8-million-character
budget can evict archived runs or older trajectory frames; the page labels
truncation. Reset and reload archive the preceding run. Recordings are specific
to this browser and origin (including port); use one recording flight-room workspace at a time. If storage is blocked, the room reports history storage unavailable.
Recordings cannot recover flights made before this feature was loaded.

`test/mission_map.test.mjs` covers ray occlusion/range, stored frames, event logs,
replay isolation from future data, read-only controls, archived reset, and
rejection of malformed storage. The new assets are included in the bridge
allowlist and package data.

The furnished layout is shared by planning, rendering and replay through
`web/room-layout.mjs`. Moving objects yield to built-in partitions and furniture
as well as custom objects. Recorded frames include room bounds and fixtures, so
older runs retain their original room dimensions during replay.
