# Live map bridge

The [operator dashboard](OPERATOR_UI.md) now has a drone-camera panel and
press-and-hold steering. The telemetry stream remains read-only; optional real
steering uses the separate [SANDO mission supervisor](SANDO_CONTROL.md), with
lease expiry, waypoint handoff and explicit readiness. Physical Pause/Hold is
still unsupported. The browser-only demo works without ROS or a camera source.

The optional [continuous frontier explorer](EXPLORATION.md) keeps selecting
destinations from timestamped raw LiDAR evidence until reachable frontiers are
exhausted within configured mission bounds. It runs through the same supervisor,
retains manual steering handoff, and distinguishes blocked from completed work.
It is opt-in and has not been validated in live Gazebo flight.

This read-only ROS 2 Humble Python package streams SANDO's existing geometric map and odometry to a bundled interactive browser viewer. It does not train a scene or wait for a recording. Open the viewer and map snapshots appear as the existing mapper publishes them; the default bridge processing/transmission target is at most 10 Hz. Actual end-to-end latency must be measured on the simulator machine.

## Run with the simulation

Run the existing Gazebo Classic SANDO pipeline and ACL mapper first. Both must publish in the same ROS domain as this bridge. The bridge needs no Gurobi or custom dynus message imports; the upstream planner still has its own dependencies.

From a terminal on the Ubuntu 22.04 / ROS 2 Humble machine:

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install 'websockets==13.1'
# From the SLAMdunk_HackTx26 repository root:
colcon build --symlink-install --base-paths integration/slam_dunk_bridge
source install/setup.bash
ros2 run slam_dunk_bridge live_map_bridge
```

Open **http://127.0.0.1:8765/?source=live** on that machine to view bridge data. The dashboard defaults to an explicitly labeled browser-only demo without this query. See [the operator UI guide](OPERATOR_UI.md) for Live/History screens, local preview, margin proposals, and test instructions. Drag to orbit, scroll to zoom, and use Fit to recenter. The coral marker shows drone pose; the lime line is received pose history, **not a planned trajectory**. Map points are colored by height, not camera RGB. Connection/freshness indicators distinguish missing or stale data. Existing geometry remains visible during a pause/disconnection. The original cyan/orange single viewer remains at `/legacy`.

Defaults are `/NX01/occupancy_grid`, `/NX01/visual_slam/odom`, target frame `map`, 20,000 displayed points, maximum 2,000,000 input points / 64 MB input cloud, eight clients, and loopback-only listening. Configure them with ROS parameters:

```bash
ros2 run slam_dunk_bridge live_map_bridge --ros-args \
  -p map_topic:=/NX01/downsampled_occupancy_grid \
  -p pose_topic:=/NX01/visual_slam/odom \
  -p map_frame:=map -p rate_hz:=5.0 -p max_points:=10000
```

The downsampled topic exists only when upstream mapper downsampling is enabled. Discover actual names/QoS with `ros2 topic list -t` and `ros2 topic info -v TOPIC`. Both subscriptions use best-effort KeepLast(1), compatible with the mapper's sensor QoS and the odometry reliable publisher. The callback retains only the newest sample; a timer processes it and records overwritten/rejected counters. Sensor/map source publication stamps are preserved. Queue drops inside DDS are not included in application counters.

To view from another device, explicitly bind `-p host:=0.0.0.0` and open `http://SIMULATOR_IP:8765/?source=live` on a trusted network, or tunnel the loopback endpoint over SSH. This local-development endpoint has no authentication or TLS. No command/control messages are accepted.

## Synthetic demonstration without ROS

With Python 3.10+ and the WebSocket dependency installed:

```bash
cd integration/slam_dunk_bridge
python3 -m pip install 'websockets==13.1'
python3 -m slam_dunk_bridge.demo
```

Open **http://127.0.0.1:8765/?source=live** (or the URL printed by the demo). This connects to the Python synthetic stream, and clearly labels **SYNTHETIC DEMO**. The default `/` runs the separate browser-only interactive demo. Neither validates Gazebo, ROS topic wiring, pose alignment, or physical navigation. Stop with Ctrl+C. `--host` and `--port` are available for this demo.

Use the same `python3` environment for this pinned dependency, colcon, and the installed ROS executable. Ubuntu 22.04's `python3-websockets` package alone may provide an older version; rosdep alone does not satisfy this package's tested WebSocket version. The bridge uses the explicit `websockets.legacy.server` API from 13.1.

## Browser/backend contract

Connect a WebSocket to `ws://127.0.0.1:8765/stream`. The same server serves the viewer at `/`. Messages are JSON with `protocol_version: 1`:

```json
{"type":"map","protocol_version":1,"operation":"replace","revision":1,"frame_id":"map","stamp":{"sec":123,"nanosec":0},"source":"/NX01/occupancy_grid","point_count":2,"points":[[1,2,3],[4,5,6]],"statistics":{"input_points":2,"sampled_points":2,"invalid_points":0}}
```

- `map`: a **complete replacement**, including an empty array to clear the map. Never append these snapshots. XYZ are meters in `frame_id`. Uniform input-index sampling bounds processing/output; this is not spatial voxel downsampling. Invalid sampled points are omitted. Revision is bridge-local and restarts with the process.
- `pose`: `frame_id`, `child_frame_id`, `stamp`, `source`, `position: [x,y,z]`, `orientation: [x,y,z,w]`. The position is the Odometry child frame's origin in the target frame. In the standard SANDO configuration this is `NX01/base_link`.
- `status`: `mode: "map_snapshot"`, target frame, seconds since each last **successfully accepted** map/pose, and received/superseded/rejected application counters. `null` age means nothing has been accepted. Synthetic demo uses `mode: "synthetic_demo"`. Browser also ages reception independently so paused ROS time does not freeze its freshness indication.

There is one latest serialized message per type. Every connected client gets the current values, including on reconnect; intermediate updates may be skipped. Slow clients are disconnected after a one-second send timeout. Per-client histories and pending map queues do not grow without bound. A map of 20,000 XYZ points can still be a substantial JSON message; lower `max_points` / `rate_hz`, or select upstream downsampling on a slow connection. Binary encoding/delta compression is deferred.

## Frames, timing and scope

- Map snapshots already in the target frame need no TF lookup. For other frames, the bridge looks up target-from-source TF at the message timestamp and rejects the update if unavailable. There is no latest-TF fallback; zero timestamps, missing frames, non-finite geometry and duplicate/out-of-order stamps are rejected.
- The mapper's timestamp is its **map publication time**, not the original raw scan capture time. Transforming this output cannot recover upstream acquisition alignment. Fixing the mapper's latest-TF fusion behavior, scan motion compensation, accurate camera poses and RGB registration remains separate work.
- The live map retains the existing mapper's finite grid bounds, filtering (including floors near ground), and clearing behavior. It is a useful live navigation visualization, **not a guaranteed persistent full-building reconstruction**. The bridge deliberately does not accumulate full map snapshots; a persistent raw-scan fusion mode is deferred.
- `visual_slam/odom` in this simulation is synthesized by fake_sim. Its name does not mean a visual SLAM estimator or independent Gazebo ground truth is running.
- Keep the default wall-time behavior for this SANDO checkout unless all upstream sources are deliberately switched to a consistent clock. If source time rewinds (simulation reset or looping bag), restart the bridge. Old timestamps are deliberately not silently accepted. Successful browser reconnection clears previous stream state, then establishes the new frame from fresh snapshots; while disconnected, old geometry remains visible.
- This MVP does not export Splatfacto datasets, color points using RGB, command destinations, perform SLAM, or replace SANDO's obstacle avoidance. Those components can use the same topics independently.

## Verification

```bash
cd integration/slam_dunk_bridge
python3 -m unittest discover -s test -v
```

Core tests exercise endian/offset/padded cloud layouts, invalid data, bounded sampling, rigid transforms, snapshot replacement/clear, latest-only transport and slow-client timeout without ROS imports. Adapter tests use explicit ROS stubs to check frame/timestamp rejection, counters and pose composition; they are not ROS integration tests.

Before a real demo, verify the actual map/pose topics, open the viewer while navigating, and confirm points and pose share a frame. Stop map publication to verify the stale indication, reconnect the viewer, and confirm empty mapper snapshots clear displayed geometry. Measure bridge CPU, network size and latency on the target machine. Live ROS/Gazebo integration remains unvalidated on the Windows development host.
