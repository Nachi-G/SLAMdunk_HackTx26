# Held steering and camera integration

The dashboard's direction buttons now temporarily override exploration while
pressed. Releasing, leaving the page, losing focus, or losing the command lease
returns destination ownership to configured autonomy: the default waypoint loop
or the optional [continuous frontier explorer](EXPLORATION.md). Neither is
direct velocity control or validated physical stopping. The local demo
illustrates the interaction without ROS.

## Components

```text
ROS map/odometry -> existing bridge :8765/stream -> browser map and history
Browser hold -> optional sidecar :8766/control -> mission supervisor
Waypoints -> mission supervisor -> /NX01/term_goal -> SANDO -> controller
ROS RGB -> web_video_server :8081/stream -> browser camera panel
```

The telemetry WebSocket stays read-only. The optional command server binds only
to loopback and admits exact configured browser origins. For another computer,
tunnel the map, command, and video ports over SSH; retain loopback bindings.

## ROS 2 Humble setup on the simulator machine

Build/source this package using the existing [bridge instructions](README.md).
The new executable is `mission_supervisor`. Its dependencies include
`geometry_msgs`, `nav_msgs`, `sensor_msgs`, `rclpy`, and pinned `websockets==13.1`.

Before enabling control:

1. Disable/remap every other `/NX01/term_goal` writer, including `goal_sender`
   and `goal_relay`. The node checks the ROS graph and waits for exactly one
   matching publisher endpoint for at least one second. Duplicate same-name
   nodes are rejected too. This admission check is not distributed mutual
   exclusion; prevent other publishers from being launched during operation.
2. Copy/edit `config/operator_supervisor.yaml`. Set `waypoints_json` to metric
   XYZ positions suitable for your actual environment, for example
   `'[[3.0, 0.0, 2.0], [3.0, 4.0, 2.0]]'`. The provided default is an empty list.
   SANDO still determines routes and can be blocked by obstacles/unknown space.
3. Verify the map/odometry frame and clock. This version rejects inputs outside
   the planning frame instead of guessing a transform. Source timestamps must
   match the node's ROS clock; monotonic wall time independently expires leases.
   Source age consumes the freshness budget, rather than restarting at receipt.
4. Match SANDO's altitude settings, map bounds and `goal_seen_radius`. Default
   `force_goal_z=true` means SANDO forces its configured flight altitude; this
   version's steering controls are horizontal and preserve requested altitude.

From this package's directory in the sourced ROS workspace:

```sh
ros2 run slam_dunk_bridge mission_supervisor --ros-args \
  --params-file config/operator_supervisor.yaml \
  -p enable_exploration:=true
```

Enabling starts waypoint navigation only after valid waypoints, fresh map/pose,
and publisher admission are available. With no configured waypoints the sidecar
stays waiting and does not accept steering. The installed configuration is also
under `share/slam_dunk_bridge/config/`.

Open the dashboard served by the map bridge:

```text
http://127.0.0.1:8765/?source=live&control=ws%3A%2F%2F127.0.0.1%3A8766%2Fcontrol
```

The static demo on port 8080 has no telemetry endpoint; selecting Live there
alone cannot connect it to ROS. Use the bridge-served URL above. Command socket
connection is explicit and never guessed from the read-only telemetry stream.

## Steering semantics

- One active operator owns a steering lease. Forward/Back/Left/Right use the
  drone heading captured when the supervisor accepts the press, so avoidance
  turns do not rotate the requested world direction unexpectedly.
- The browser renews every 100 ms while pressed. The sidecar protocol requires
  a 500 ms lease. A monotonic watchdog in the ROS process runs independently of
  the browser and command socket. Expiration withdraws the manual override.
- SANDO receives a rolling destination, default 3 m ahead. Lookahead must exceed
  `goal_seen_radius` by more than 0.1 m. Goal updates are limited to 5 Hz and
  require at least 0.15 m of position progress. A request does not force passage
  through an occupied/unknown corridor or bypass obstacle inflation.
- Release/disconnect/expiry actively replace the manual destination with a
  newly selected autonomous waypoint. Merely stopping publications would leave
  the last manual goal active, so the supervisor does not use that approach.
- Epochs, lease IDs, sequence numbers, request correlation and cancel-before-ack
  handling prevent released or expired presses from being revived by delayed
  traffic. Reconnect requires a new physical press; it never reacquires control.
- If telemetry becomes stale or another publisher appears, authority is
  withdrawn and status becomes waiting/fault. The last SANDO goal may still
  execute: this is **not** a braking command. A physical controller watchdog
  and the separate planner Stop/Hold extension remain necessary for those faults.

The center button toggles Pause/Resume in the synthetic demo. The real sidecar
advertises `pause:false`, so it is disabled in live mode. SANDO has no supplied
operator braking/hold service; this implementation does not pretend that a
current-position goal or stopping publication physically halts the drone.

## Camera

The upper-right Live panel now displays a configurable external RGB image
stream. The combined occupancy map remains on History. No camera source means
an explicit offline placeholder, including in the synthetic demo.

The checkout configures `/NX01/d435/color/image_raw`; verify actual topic names,
QoS and image publication with the running simulator. A compatible MJPEG
provider is [RobotWebTools web_video_server](https://github.com/RobotWebTools/web_video_server).
Its documented ROS 2 setup supports:

```sh
sudo apt install ros-${ROS_DISTRO}-web-video-server
ros2 run web_video_server web_video_server --ros-args \
  -p port:=8081 -p address:=127.0.0.1
```

Paste the following into **Camera source** and choose **Connect feed**:

```text
http://127.0.0.1:8081/stream?topic=/NX01/d435/color/image_raw&type=mjpeg
```

Alternatively supply a URL-encoded `camera` query parameter to the dashboard.
Use the direct multipart stream endpoint, not its HTML stream-viewer page.
HTTP(S) image URLs are accepted; embedded credentials, fragments and HTTPS-to-
HTTP mixed content are rejected. Stream resources detach on History, hidden
pages, explicit disconnect and dashboard disposal. MJPEG image-load events
are not per-frame metadata: the UI labels capture age/freshness unavailable,
and does not infer FPS or fresh camera data from the first successful load.

## Control protocol

All messages use `protocol_version:1`. Client messages:

| Type | Required payload |
| --- | --- |
| `steer_start` | unique `request_id`, `direction`, current `epoch` |
| `steer_renew` | `lease_id`, increasing `seq`, current `epoch` |
| `steer_release` | `lease_id`, increasing `seq`, current `epoch` |
| `steer_cancel` | original start `request_id`, its `epoch` |
| `pause` / `resume` | unsupported by this real backend; rejected explicitly |

`control_status` replies/heartbeats include epoch, mode, readiness, capability
flags, lease ID, active request ID, last request ID, accepted result, reason
and TTL. Frontend state changes follow server acknowledgment. One socket cannot
release another operator's lease. Messages/client count, JSON size and inbound
rate are bounded. Origin checks are local-development protections, not a
multi-user authentication system.

## Verification and limits

```sh
node test/run_ui_tests.mjs
python -m unittest discover -s test -v
node test/control_contract_smoke.mjs python
```

The last command launches a temporary Python sidecar with a fake destination
sink and connects the actual JavaScript adapter over real sockets. It validates
start/renew/release/expiry and autonomous handoff across the language boundary.
ROS stubs check message types, float setters, frame admission and duplicate
publisher rejection. Neither these tests nor the synthetic UI validate Gazebo
flight, actual RGB delivery, planner feasibility, controller failure handling,
or runtime margin application. See [the test report](test/UI_TEST_REPORT.md).
