Pinned upstream dependencies from SANDO gitlinks:

- DecompROS2: https://github.com/kotakondo/DecompROS2/tree/a2bc42ed46bf934323994706f17af8a3883860ea
- dynus_interfaces: https://github.com/kotakondo/dynus_interfaces/tree/f5697027bf87c2b940f91959d11a49cd6cfe6b58

Original license files and source notices are retained.

Car build adaptations: DecompUtil package metadata names cmake/eigen instead of ROS 1 catkin. Unused RViz/test-node packages have COLCON_IGNORE; the unchanged data_ros_utils.hpp header is copied into the isolated planner include tree so that building the car planner does not require the RViz plugin binaries.
