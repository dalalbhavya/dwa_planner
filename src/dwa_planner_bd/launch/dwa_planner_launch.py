import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generates the launch description for starting nodes."""

    # Create the launch description object.
    ld = LaunchDescription()

    # Package share directories.
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_dwa_planner_bd = get_package_share_directory('dwa_planner_bd')


    # paths
    rviz_config_path = os.path.join(pkg_dwa_planner_bd, 'rviz', 'dwa.rviz')
    world_file_path = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.world')

    set_turtlebot_model_env_var = SetEnvironmentVariable(
        'TURTLEBOT3_MODEL', 'burger'
    )

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
        launch_arguments={'world': world_file_path}.items()
    )
    
    # state publisher for robot
    launch_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # RViz configuration
    launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # spawn turtlebot3 in gazebo
    spawn_turtlebot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf'),
            '-x', '0.5',
            '-y', '0.5',
            '-z', '0.3'
        ],
        output='screen'
    )

    ld.add_action(set_turtlebot_model_env_var)
    ld.add_action(launch_gazebo)
    ld.add_action(launch_robot_state_publisher)
    ld.add_action(spawn_turtlebot_node)
    ld.add_action(launch_rviz)
    ld.add_action(node_dwa)

    return ld