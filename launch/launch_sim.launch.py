import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    package_name = 'diffdrive_bot'
    file_subpath = 'description/robot.urdf.xacro'

    # Use xacro to process the file [Not needed, rsp.launch.py already contains a similar function]
    # xacro_file = os.path.join(get_package_share_directory(package_name),file_subpath)
    # robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Defining world
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacles.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # Launch robot_state_publisher using our own prepared launch file 'rsp.launch.py' (another method is to simply run the node)
    # Ensure sim time is enabled
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py')]),
            launch_arguments = {'use_sim_time':'true'}.items()
        
    )

    # Launch Gazebo
    # Include launch file for Gazebo,  found in ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments = {'gz_args': ['-r -v4 --render-engine ogre ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn robot model
    # Spawner node taken from ros_gz_sim package
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic','robot_description',
                                   '-name','diffdrive_bot',
                                   '-z','0.1'],
                        output='screen')
    

    # Create the bridge node for gz_bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ]
    )


    # Run the nodes
    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        ros_gz_bridge
    ])