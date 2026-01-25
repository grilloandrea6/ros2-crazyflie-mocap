import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('crazyflie_simulator'),
                'launch',
                'simulator.launch.py'
            )
        )
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('crazyflie_viz'),
                'launch',
                'visualization.launch.py'
            )
        )
    )
   
    launch_desc = [
        simulator_launch,
        visualization_launch,
    ]

    return LaunchDescription(launch_desc)
