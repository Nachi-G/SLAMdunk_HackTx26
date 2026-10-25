# PX4 X500 forest runtime and live SLAMdunk dashboard

Separate Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic runtime, preserving the
repository's existing Classic/Humble integration. Uses PX4 Offboard for the X500,
known forest geometry, and scripted moving obstacles. **SANDO and live LiDAR are
not driving this demonstration.** Stationary LiDAR perception and read-only SANDO
trajectory/watchdog work are included; full sensor-driven SANDO flight is pending.

Includes forest/LiDAR/contact worlds, baseline/slalom/twelve-object challenge,
Start, Return Home, leased manual override, pinned goals, global/local prediction
overlays, measured speed, clear landing alignment, and checked low-speed recovery
from the extra obstacle buffer. Historical diagnostics and their original ~/hacktx
paths are retained as evidence, not portable launch instructions.

## Existing installation

Requires an already configured PX4/Jazzy/Harmonic environment. Defaults:

```bash
export PX4_SOURCE_DIR="$HOME/PX4-Autopilot"
export HACKTX_WORKSPACE="$HOME/hacktx"
export PX4_MSGS_WS="$HACKTX_WORKSPACE/px4_ros2_ws"
cd simulation/px4_harmonic
```

Match px4_msgs to the firmware. Development used release/1.18 definitions with
locally synchronized messages. Dependencies, builds, logs and licenses are not
bundled. The obstacle helper builds automatically from C++ source on first demo
startup, using c++, pkg-config, gz-transport13 and gz-msgs10. Build it before tests:

```bash
bash scripts/build_obstacle_rpc.sh
```

SANDO adaptations are captured in patches/sando-harmonic.patch against MIT ACL
SANDO commit 1a738df919f9abe60cee3c1afbecd72579da8f6f. The matching configuration is
config/sando_nx01.yaml; upstream licensing is retained in patches/SANDO_LICENSE.
Applying/building that patch is separate from running this PX4 demonstration.

## Launch in separate WSL tabs

Run each command from this runtime directory and leave its terminal running.
Stop old controllers/simulators only after verified landing/disarming.

```bash
bash scripts/start_solid_forest.sh
MicroXRCEAgent udp4 -p 8888
bash scripts/start_px4_lidar.sh
# Open QGroundControl and wait for its vehicle heartbeat.
bash scripts/start_live_dashboard.sh
bash scripts/start_dashboard_controller.sh
```

Open http://localhost:8765/?source=live&control=ws://localhost:8765/control .
The dashboard prepares an isolated system-site-packages .dashboard_venv with
websockets 13.1. The standby controller stays grounded until Start is accepted.
Start requires fresh telemetry, QGC, calibration, a checked route, and a landed /
disarmed drone. Return Home routes to the recorded takeoff location or a nearby
clear landing site. Avoid concurrent PX4 setpoint publishers.

Place pin then click either map. Blocked pins may resolve to a reachable clear
point within 2.5 m, acknowledged with the actual goal. Blue is global route,
orange is local prediction, and lime is pose history. Held manual control bypasses
avoidance in simulation; release attempts to rejoin. Contacts latch a failed run.
Battery is the fixed 97% UI value requested for the demo, not measured charge;
speed is PX4's estimated 3D velocity magnitude. Flight requires user authorization.
No drone teleportation is used. These tools are not a hardware flight deployment.

## Tests and verification limits

```bash
bash scripts/build_obstacle_rpc.sh
python3 -m unittest discover -s tests
node tests/test_dashboard_control_reconnect.mjs
node tests/test_dashboard_plan_ui.mjs
# From repository root:
node integration/slam_dunk_bridge/test/run_ui_tests.mjs
```

Baseline/challenge host flights completed during development; generated runtime
logs are kept outside Git. The latest UI lifecycle, pin changes, landing alignment
and buffer recovery have local test coverage; their combined live PX4 run remains
to be verified. Unit/DOM tests do not prove flight, collision response, rendering
layout or real-world safety. This README supersedes the older workspace-specific
launch paths in the included component notes.
