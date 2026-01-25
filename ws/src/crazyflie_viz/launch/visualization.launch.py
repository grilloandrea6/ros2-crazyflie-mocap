import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('crazyflie_viz'),
        'gui',
        'rvizConfig.rviz'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config],
        output='screen'
    )

    rviz_interface_node = Node(
        package="crazyflie_viz",
        executable="crazyflie_rviz_interface",
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        rviz_interface_node,
    ])
