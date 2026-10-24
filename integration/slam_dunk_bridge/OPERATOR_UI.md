# Operator dashboard

Local, buildless browser UI. No Gazebo, ROS, npm install, CDN, or API key is
needed to inspect the synthetic mission. It is a UI prototype, not a validated
flight-control implementation.

## Open locally

From `integration/slam_dunk_bridge` (or `hacktx-transfer/integration/slam_dunk_bridge`
in the parent SANDO checkout):

```sh
python -m http.server 8080 --bind 127.0.0.1 --directory slam_dunk_bridge/web
```

Open **http://127.0.0.1:8080/**. The synthetic mission is visibly labeled. The
Live screen contains the top view, manual controls, speed, margin profile,
main 3D view, drone camera panel, battery, and flight time. History
contains the combined-site recorded map, replay cursor, playback, and details. Orbit,
zoom, recenter, follow, expand, and keyboard controls work locally.

The demo and its in-memory history restart when the page refreshes. History is
bounded and has no disk persistence. An accumulated occupancy sample is not
an authoritative global map: old obstacles can remain in this view, occupied
snapshots do not establish free space, and eviction can remove old geometry.

## Connect the existing bridge

The Python bridge serves this dashboard at `/`; the original viewer remains
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
Synthetic mode only previews values. Speed preference is separate from the
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
