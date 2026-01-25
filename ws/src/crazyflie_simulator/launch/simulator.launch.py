import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    simulator_node = Node(
        package="crazyflie_simulator",
        executable="simulator_node",
        output='screen'
    )

    return LaunchDescription([
        simulator_node,
    ])
