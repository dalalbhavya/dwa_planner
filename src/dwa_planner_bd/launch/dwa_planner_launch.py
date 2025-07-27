import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generates the launch description for starting nodes."""

    # Create the launch description object.
    ld = LaunchDescription()

    # Package share directories.
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Define the world file path.
    world_file_path = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.world')

    # Define the node to launch.
    node_dwa = Node(
        package='dwa_planner_bd',
        executable='dwa_planner',
        output='screen',
        emulate_tty=True,
        parameters=[]
    )

    # launch gazebo node
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # Pass the world file to Gazebo.
        launch_arguments={'world': world_file_path}.items()
    )
    
    
    ld.add_action(launch_gazebo)
    ld.add_action(node_dwa)

    return ld