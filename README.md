# SLAMdunk_HackTx26

Disaster reconnaissance drone prototype built around SANDO's ROS 2 / Gazebo Classic pipeline.

New agents: start with the [project handoff](AGENT_HANDOFF.md) for goals, completed work, architecture, limitations, verification evidence, and next steps.

The [furnished house test environment](simulation/house/README.md) provides a standalone Gazebo Harmonic (Gazebo Sim 8) world with eight rooms, furniture, two moving patrol obstacles, and a ceiling fan. Launch it with `bash simulation/house/run_house.sh` on a Harmonic host; a static baseline is included. The existing SANDO robot integration still targets Gazebo Classic.

The [live map bridge and browser viewer](integration/slam_dunk_bridge/README.md) expose the existing ROS map and drone pose with bounded updates. A synthetic demo runs without ROS; the ROS adapter targets the existing simulation topics.

Start with the [sensor and reconstruction audit](docs/sando-sensor-reconstruction-entrypoints.md) and [selected SANDO files and runtime setup](integration/README.md). The first milestone is live geometric mapping and operator goals; offline Nerfstudio/Splatfacto reconstruction follows verified RGB capture and camera poses.

The [operator dashboard](integration/slam_dunk_bridge/OPERATOR_UI.md) combines Nachi's Live/History layout with Rishi's SANDO-inspired browser simulation at `/` (also `/dashboard.html`). Choose targets, add static or moving objects, change their X/Y motion, and inspect planned routes, predictions and simulated POV alongside steering, camera and history. **Open furnished flight room** leads to the preserved 32 × 24 m editor at `/flight-room.html`, including locally saved mission map/replay. Live mode connects the ROS bridge and optional leased supervisor, and displays frontier exploration metrics when supplied. Follow the [SANDO control setup](integration/slam_dunk_bridge/SANDO_CONTROL.md); physical Stop/Hold remains separate work. See the [Backup integration notes](docs/backup-integration.md) for launch and validation details.

The opt-in [continuous frontier explorer](integration/slam_dunk_bridge/EXPLORATION.md) uses persistent raw-sensor voxel evidence to select new SANDO destinations until reachable frontiers are exhausted within configured mission bounds. It supports horizontal or 3D exploration, bounded retries, completion confirmation, and manual steering handoff. Live ROS/Gazebo flight validation remains required.
