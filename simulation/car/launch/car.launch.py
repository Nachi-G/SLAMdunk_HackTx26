"""Harmonic + ROS bridge + SLAM; optional isolated SANDO planner."""
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,IncludeLaunchDescription,OpaqueFunction,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=Path(get_package_share_directory('slamdunk_car'))
    mode=LaunchConfiguration('mode').perform(context)
    gui=LaunchConfiguration('gui').perform(context).lower()=='true'
    slam=LaunchConfiguration('slam').perform(context).lower()=='true'
    explore=LaunchConfiguration('explore').perform(context).lower()=='true'
    if mode not in ('teleop','sando'):raise ValueError('mode must be teleop or sando')
    if (mode=='sando' and not slam) or (explore and mode!='sando'):raise ValueError('SANDO requires SLAM; exploration requires SANDO mode')
    world=Path(LaunchConfiguration('world').perform(context))
    if not world.is_file():raise FileNotFoundError(world)
    actions=[ExecuteProcess(cmd=['gz','sim','--force-version','8','-r']+([] if gui else ['-s'])+[str(world)],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='car_gz_bridge',parameters=[{'config_file':str(share/'config/bridge.yaml'),'use_sim_time':True}],output='screen'),
        Node(package='slamdunk_car',executable='drive_guard',parameters=[{'use_sim_time':True,'mode':mode,'require_exploration_gate':explore}],output='screen')]
    # DiffDrive alone owns odom -> base_link; SLAM owns map -> odom.
    for child,x,y,z,roll,pitch,yaw in [('laser',0,0,.6,0,0,0),('lidar3d',0,0,.6,0,0,0),('imu_link',0,0,.2,0,0,0),('camera_link',.28,0,.315,0,0,0),('camera_optical',.28,0,.315,-1.5707963267948966,0,-1.5707963267948966)]:
        actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name='car_tf_'+child,
            arguments=['--x',str(x),'--y',str(y),'--z',str(z),'--roll',str(roll),'--pitch',str(pitch),'--yaw',str(yaw),'--frame-id','base_link','--child-frame-id',child],parameters=[{'use_sim_time':True}]))
    if slam:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('slam_toolbox'))/'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time':'true','slam_params_file':str(share/'config/slam.yaml')}.items()))
    if mode=='sando':
        planner=Path(get_package_share_directory('car_sando_planner'))
        manifest=json.loads((planner/'source_manifest.json').read_text())
        if manifest.get('adaptation')!='CAR_PLANAR_V1':raise ValueError('Build the isolated planar planner included in this folder')
        parameters=yaml.safe_load((planner/'config/sando.yaml').read_text())['sando_node']['ros__parameters']
        parameters.update(yaml.safe_load((share/'config/sando_car.yaml').read_text())['sando_node']['ros__parameters'])
        actions.extend([Node(package='car_sando_planner',executable='sando',namespace='car_sando',name='sando_node',parameters=[parameters],output='screen'),
            Node(package='slamdunk_car',executable='sando_adapter',parameters=[{'use_sim_time':True}],output='screen')])
        if explore:actions.append(Node(package='slamdunk_car',executable='car_explorer',parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory('slamdunk_car'))
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('slam',default_value='true'),
        DeclareLaunchArgument('mode',default_value='teleop'),
        DeclareLaunchArgument('explore',default_value='false'),
        DeclareLaunchArgument('domain_id',default_value='20'),
        DeclareLaunchArgument('world',default_value=str(share/'worlds/house_car.sdf')),
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('domain_id')),
        OpaqueFunction(function=setup)])
