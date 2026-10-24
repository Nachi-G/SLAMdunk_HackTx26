# SANDO integration starting point

Read [the sensor audit](../docs/sando-sensor-reconstruction-entrypoints.md) first. Build a ROS 2 adapter alongside the existing planner: expose the ACL mapper's occupied PointCloud2 output for the first live UI, and record original RGB/depth/LiDAR plus timestamped camera poses for later reconstruction.

## Selected upstream files

`sando_reference/` contains unmodified source copies selected from MIT ACL SANDO commit `1a738df919f9abe60cee3c1afbecd72579da8f6f`:

- `scripts/run_sim.py`: actual sensor/mapper/planner orchestration and `--d435` switch.
- `scripts/bag_record.py`: existing recording implementation to extend; its default topic set lacks the full calibration/depth capture contract.
- `launch/base_sando.launch.py`, `launch/onboard_sando.launch.py`: Gazebo Classic and model/planner wiring.
- `urdf/quadrotor.urdf.xacro`, `_d435.urdf.xacro`, `_d435.gazebo.xacro`: active LiDAR/RGB-D mounts, camera TF chain and sensor settings.
- `src/sando/fake_sim.cpp`, `src/sando/sando_node.cpp`: pose generation, command interface and map consumption.
- `config/sando.yaml`, `package.xml`, `CMakeLists.txt`, `.gitmodules`, `README.md`: upstream configuration, dependency and installation references.

This slice is for reference and adaptation, not a standalone ROS package. `COLCON_IGNORE` prevents accidental discovery as a second incomplete `sando` package. Sensor meshes, plugin sources, mapper implementation, planner headers, generated interfaces and other build dependencies stay in the pinned runtime below. No optimizer or planner implementation has been recreated.

## Runtime source

`vendor/sando` is a Git submodule pinned to the same commit. Initialize it from this repository's root:

```bash
git submodule update --init --recursive vendor/sando
```

Follow `vendor/sando/README.md` for Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic and Gurobi setup. ACL mapping and sensor plugins are nested upstream dependencies; some dependencies need additional credentials or system packages. This checkout has not been built or launched as part of the source transfer. Keep runtime edits in the submodule or a fork; copied reference files are not installed automatically.

## Ownership and licensing

The project root LICENSE remains unchanged. SANDO's BSD-3-Clause notice is preserved at `sando_reference/LICENSE`; D435 Xacros retain their Intel/PAL Robotics copyright and Apache-2.0 headers, with the Apache license text alongside them. Other dependency licenses remain in the initialized upstream submodules. This reference transfer preserves original source notices and does not relicense upstream code under this project's root license.
