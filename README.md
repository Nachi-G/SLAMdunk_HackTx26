# SLAMdunk_HackTx26

Disaster reconnaissance drone prototype built around SANDO's ROS 2 / Gazebo Classic pipeline.

The [live map bridge and browser viewer](integration/slam_dunk_bridge/README.md) expose the existing ROS map and drone pose with bounded updates. A synthetic demo runs without ROS; the ROS adapter targets the existing simulation topics.

Start with the [sensor and reconstruction audit](docs/sando-sensor-reconstruction-entrypoints.md) and [selected SANDO files and runtime setup](integration/README.md). The first milestone is live geometric mapping and operator goals; offline Nerfstudio/Splatfacto reconstruction follows verified RGB capture and camera poses.
