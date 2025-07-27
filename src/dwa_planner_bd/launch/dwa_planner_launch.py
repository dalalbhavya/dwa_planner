from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generates the launch description for starting nodes."""

    # Create the launch description object.
    ld = LaunchDescription()

    # Define the node you want to launch.
    node_dwa = Node(
        package='dwa_planner_bd',
        executable='dwa_planner',
        output='screen',
        emulate_tty=True,
        parameters=[]
    )

    # Add the node to the launch description.
    ld.add_action(node_dwa)

    return ld