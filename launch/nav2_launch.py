import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def generate_launch_description():

    nav_pkg = get_package_share_directory('nav2_bringup')
    this_pkg = get_package_share_directory('diffdrive_bot')
    # map_pkg = get_package_share_directory('slam')

    ekf_params = os.path.join(this_pkg, 'config', 'ekf.yaml')
    nav_params = os.path.join(this_pkg, 'config', 'nav_params.yaml')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(this_pkg, 'config' 'ekf.yaml'), {'use_sim_time': True}],
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav_params,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        robot_localization_node,
        nav_launch
    ])