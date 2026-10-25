### This is ported version for ROS2 humble of https://github.com/sikang/DecompROS 

#### Introduced lifetime for decomp_ros_msgs::msg::PolyhedronArray message type 

#### Dependencies

    pip install empy==3.3.4 catkin_pkg lark

#### To build 

    colcon build

#### To test 
    
    ros2 launch decomp_test_node test_decomp_rviz_launch.py

#### Errors and solutions

##### colcon build failure

    Error: Could not find a package configuration file provided by "decomp_util" with any of the following names:

        decomp_utilConfig.cmake
        decomp_util-config.cmake

    Solution:

        source install/setup.bash

##### rosidl_typesupport_c-related errors

    Error: rosidl_generator_py.import_type_support_impl.UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c' for package 'decomp_ros_msgs'

    Solution: (ref: https://github.com/ros2/examples/issues/303)

        If you are using Conda, probably it's better to stop using it, and build workspace without it. Also it is suggested using the same python version as ROS2's python version.
