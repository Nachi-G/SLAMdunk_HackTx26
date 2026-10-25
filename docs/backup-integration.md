# Backup integration

`Backup` starts from Rishi's `e77dd9b` and merges remote main `0a0cdd9`,
which includes Nachi's `1a67e5f`. The original working tree is preserved;
the integration is checked out in `.worktrees/Backup`.

## Browser entry points

- `/` and `/dashboard.html`: Nachi's mission dashboard with Rishi's browser
  routing, cubic movement, dynamic obstacle predictions and simulated POV.
  The default view remains the 3D map, with camera, status and history panels.
- **Build arena**: select Goal, Obstacle or Remove and click the map. Objects
  can be crates, barriers or moving objects with an X/Y placement axis. Select
  an added object with the Goal tool to change its motion. Built-in patrols
  are retained when clearing user objects. New targets resume paused simulation.
- `/flight-room.html`: preserved furnished editor, altitude controls, manual
  pushing and bounded local mission recordings/replay.
- `/mission-map.html`: redirects to `/flight-room.html#mission-map`.
- `/?source=live&control=ws%3A%2F%2F127.0.0.1%3A8766%2Fcontrol`: live
  telemetry/control. Simulation editing is unavailable in live mode. The
  separate command socket retains leases and capability gating.

For a browser-only preview, from `integration/slam_dunk_bridge`:

```sh
python -m http.server 8080 --bind 127.0.0.1 --directory slam_dunk_bridge/web
```

Open `http://127.0.0.1:8080/`. No ROS, Gurobi, npm build or external assets
are required for the browser simulation.

For the Python synthetic telemetry backend, use `python -m slam_dunk_bridge.demo`
with `websockets==13.1`, then open `http://127.0.0.1:8765/?source=live`.
That feed is also synthetic; it is distinct from the browser planner.

## Backend preservation

Rishi's raw-sensor frontier explorer and ROS supervisor remain intact. The
dashboard now validates and retains the optional exploration status and displays
frontier/reachable/unknown counts, current goal, explanation and completion scope.
Disconnecting clears those metrics. No ROS topics are written by browser canvas
clicks, and live Pause is disabled when the backend lacks a physical hold service.

The house, car, planar SANDO planner, SLAM adapters, guarded drive and ground
exploration are preserved. The car remains a separately launched ROS application;
this integration does not add a browser wheel-command or car stop adapter.
The existing read-only bridge can still be configured for its map and odometry.

## Validation

Run from `integration/slam_dunk_bridge`:

```sh
python -m unittest discover -s test -v
node test/run_ui_tests.mjs
node test/control_contract_smoke.mjs python
# Against a running demo, optionally with --port 8794:
python test/smoke_dashboard.py 8794
```

The UI integration tests default to 300 scenarios per repeated UI flow. For a
bounded smoke run, set `SLAMDUNK_UI_SCENARIOS=8`; model/adapter, held-steering,
planner, collision and room/replay suites retain their original coverage.
The new Backup scenarios exercise real dashboard handlers for moving-object
placement, selection, axis changes and removal, paused target selection, live
exploration validation, disconnection and entry-point preservation.

Verified on Windows on 2026-10-25 with Python 3.15.1, websockets 13.1 and Node
24.22.1:

- 51 Python bridge/control/exploration/HTTP tests passed.
- JavaScript runner passed with `SLAMDUNK_UI_SCENARIOS=8`: 8,403 model/adapter
  cases, 3,000 held-steering cases, eight repetitions of each dashboard,
  release, camera and scene flow, plus planner, arena/POV, room, replay and
  combined-dashboard checks. Collision coverage included 33 scenarios and
  141,000 swept checks. The exhaustive 300-repeat UI variant was not completed
  against the final files.
- Real HTTP assets plus map/pose/status WebSocket smoke passed on port 8794.
- Real JavaScript adapter to Python command-sidecar contract passed.
- All 31 preserved Python car tests passed.
- `git diff --check` passed.

Run the preserved car checks from `simulation/car`:

```sh
python -m unittest discover -s test -v
```

These checks use synthetic data, canvas/DOM doubles and ROS stubs. They do not
validate ROS/Gazebo flight, Linux planner builds or physical stopping. Browser
visual review was attempted but computer-use approval timed out. The browser
planner approximates SANDO using inflated A*, cubic segments and temporal checks;
it does not execute the ROS/Gurobi optimizer. Drone and moving-object motion is
synthetic, including actors yielding near the vehicle.
