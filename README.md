# SLAMdunk_HackTx26

Disaster reconnaissance drone prototype built around SANDO's ROS 2 / Gazebo Classic pipeline.

New agents: start with the [project handoff](AGENT_HANDOFF.md) for goals, completed work, architecture, limitations, verification evidence, and next steps.

The [furnished house test environment](simulation/house/README.md) provides a standalone Gazebo Harmonic (Gazebo Sim 8) world with eight rooms, furniture, two moving patrol obstacles, and a ceiling fan. Launch it with `bash simulation/house/run_house.sh` on a Harmonic host; a static baseline is included. The existing SANDO robot integration still targets Gazebo Classic.

The [live map bridge and browser viewer](integration/slam_dunk_bridge/README.md) expose the existing ROS map and drone pose with bounded updates. A synthetic demo runs without ROS; the ROS adapter targets the existing simulation topics.

Start with the [sensor and reconstruction audit](docs/sando-sensor-reconstruction-entrypoints.md) and [selected SANDO files and runtime setup](integration/README.md). The first milestone is live geometric mapping and operator goals; offline Nerfstudio/Splatfacto reconstruction follows verified RGB capture and camera poses.

The [operator dashboard](integration/slam_dunk_bridge/OPERATOR_UI.md) provides Live/History screens, a configurable RGB camera panel, and held steering. Follow the [SANDO control setup](integration/slam_dunk_bridge/SANDO_CONTROL.md) for the optional leased mission supervisor; the [physical Stop/Hold design](docs/operator-push-stop-ros-design.md) remains separate work. Validation and runtime limits are recorded in the [test report](integration/slam_dunk_bridge/test/UI_TEST_REPORT.md).
