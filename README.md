# SLAMdunk_HackTx26

Disaster reconnaissance drone prototype built around SANDO's ROS 2 / Gazebo Classic pipeline.

The [furnished house test environment](simulation/house/README.md) provides a standalone Gazebo Harmonic (Gazebo Sim 8) world with eight rooms, furniture, two moving patrol obstacles, and a ceiling fan. Launch it with `bash simulation/house/run_house.sh` on a Harmonic host; a static baseline is included. The existing SANDO robot integration still targets Gazebo Classic.

The [live map bridge and browser viewer](integration/slam_dunk_bridge/README.md) expose the existing ROS map and drone pose with bounded updates. A synthetic demo runs without ROS; the ROS adapter targets the existing simulation topics.

Start with the [sensor and reconstruction audit](docs/sando-sensor-reconstruction-entrypoints.md) and [selected SANDO files and runtime setup](integration/README.md). The first milestone is live geometric mapping and operator goals; offline Nerfstudio/Splatfacto reconstruction follows verified RGB capture and camera poses.

The [operator dashboard](integration/slam_dunk_bridge/OPERATOR_UI.md) now opens a 32 × 24 m furnished 3D flight room: add static or moving objects, place destinations, inspect obstacle heat and flight tracking, and hold controls to push the drone. It uses a SANDO-inspired browser planning approximation. Use **Mission map & replay** to review the simulated scan map, flown path, timeline and event log; runs are saved locally across resets. The previous Live/History operator interface remains at `/dashboard.html`. Follow the [SANDO control setup](integration/slam_dunk_bridge/SANDO_CONTROL.md) for the optional leased mission supervisor; the [physical Stop/Hold design](docs/operator-push-stop-ros-design.md) remains separate work. Validation and runtime limits are recorded in the [test report](integration/slam_dunk_bridge/test/UI_TEST_REPORT.md).

The opt-in [continuous frontier explorer](integration/slam_dunk_bridge/EXPLORATION.md) uses persistent raw-sensor voxel evidence to select new SANDO destinations until reachable frontiers are exhausted within configured mission bounds. It supports horizontal or 3D exploration, bounded retries, completion confirmation, and manual steering handoff. Live ROS/Gazebo flight validation remains required.
