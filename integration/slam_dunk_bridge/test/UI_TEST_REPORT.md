# Operator dashboard verification - 2026-10-24

## Executed checks

Windows development host: Node 24.22.1, Python 3.15.1, websockets 13.1.
Commands run from `integration/slam_dunk_bridge`:

```sh
node test/run_ui_tests.mjs
python -m unittest discover -s test -v
node test/control_contract_smoke.mjs python
python test/smoke_dashboard.py 8794
```

| Suite | Passing cases | Coverage |
| --- | ---: | --- |
| Original model/demo/adapter | 8,403 | 28 behavior categories x 300 plus three regressions |
| Held steering state/adapter | 3,000 | 10 categories x 300: heading, TTL, pause, live status, pending cancel, source isolation, gateway restart and foreign leases |
| Mounted dashboard | 300 | Real app handlers, model, demo, adapter and renderer with DOM/canvas doubles |
| Release-event scenarios | 300 | Pointer up/cancel/lost capture, hidden document, window blur, pagehide, History switch and keyboard release |
| Camera lifecycle | 300 | Valid/invalid URLs, unknown freshness, delayed callbacks, unavailable feed, hide/show/disconnect/disposal |
| Scene lifecycle | 300 | Varying geometry, dimensions, DPR, keyboard, pan/orbit/zoom/follow/fit/cleanup |
| Invalid scene construction | 300 | Invalid canvas rejection |
| Python unittest | 26 methods | Existing bridge plus supervisor, real command sockets, ROS stubs, routing and source freshness |
| Cross-language control smoke | 1 real socket session | Actual JS adapter to Python sidecar; fake destination sink verifies steering, renewal, release, expiry and return to waypoint |
| HTTP/telemetry smoke | 10 HTTP requests + 1 WebSocket session | All dashboard modules including camera, legacy viewer and protocol-v1 map/pose/status |

JavaScript total: **12,903 named/parameterized cases**. Several categories repeat
invariants over generated inputs; this is not 12,903 distinct features or real
flights, nor hundreds of distinct tests for every internal helper/callback.

Within the 26 Python test methods are **2,700 generated supervisor scenarios**,
**300 real command-socket scenarios**, and **300 ROS-stub scenarios**. Routing
adds 360 successful asset/query variants, 400 rejected path variants and 200
upgrade/legacy scenarios. These are separate from the JS total and do not
inflate the count of Python test methods.

## Integration assertions

- Live contains the local top view, manual controls, speed and margins, main3D,
  camera, battery and time. Combined occupancy and timeline remain on History.
- Directional press temporarily overrides exploration. Release, cancellation,
  lost focus, hidden page, History/source change and disposal remove the press.
  While held, renewals run at100ms; the backend expires the lease after500ms.
- Heading is captured at acceptance. Manual goals remain beyond SANDO's
  goal-seen radius and refresh only at bounded rate after position progress.
- Release, disconnect and expiry replace the manual destination with an
  autonomous waypoint; stale telemetry reports waiting without a false physical
  stop claim. Invalid messages do not refresh successful-source age.
- Already-old source messages consume their existing freshness budget at
  receipt, rather than receiving another full validity interval.
- Pending acquisition cancellation, delayed traffic, old socket callbacks,
  lower epochs, foreign leases and gateway restarts cannot reactivate a press.
- ROS adapter startup is opt-in, requires waypoints and fresh aligned inputs,
  and admits exactly one matching goal publisher. Duplicate same-name endpoints
  are rejected. Strict float setters and invalid cloud/frame admission are tested.
- Real Pause is unavailable because SANDO has no braking/hold service; synthetic
  Pause/Resume is distinct from release-to-explore. Margins remain proposals.
- Camera is an external RGB image stream, never generated map footage. It shows
  offline without a source and makes no per-frame freshness inference from
  MJPEG load events. History/hidden/disconnect/disposal detach stream resources.
- Existing telemetry stays read-only. Optional command connection is explicit,
  origin-restricted and bounded, and never silently falls back to synthetic data.

## Review process

Frontend state and backend implementation ran in parallel with root's UI and
camera work, overseen by a separate orchestrator. After a temporary agent usage
limit interruption, user-requested continuation resumed independent logic and
code reviews. Root fixed the findings and completed integration verification.
Final oversight checks include the completed documentation and packaged config.

Review fixes include failed-renewal pressed-state cleanup, stale status send
ordering, source-age accounting, duplicate publisher admission, fixed500ms wire
contract, ROS float setters, pending-cancel races and explicit live capabilities.

## Verification limits

Actual browser visual inspection remains unavailable after Computer Use stopped
because it could not reliably determine Chrome's current URL to enforce policy.
No alternate browser automation was attempted. DOM/canvas doubles do not verify
CSS layout, typography, native fullscreen, browser accessibility or actual frame
rate. The user has visually inspected the earlier UI; these latest changes have
not received an automated visual browser review.

Gazebo/ROS runtime, real RGB delivery, physical braking, planner feasibility,
hardware-controller failure handling and runtime margin application remain
unvalidated. ROS tests use stubs; the real-socket control smoke uses a fake goal
sink. Autonomous behavior is configured waypoint navigation, not frontier
exploration or coverage. Persistent reconstruction, camera recording and dynamic
object tracking are outside this implementation.
