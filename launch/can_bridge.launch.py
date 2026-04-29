from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    can_bridge_node = Node(
        package='can_bridge',
        executable='can_bridge',
        name='can_bridge',
        output='screen'
    )

    return LaunchDescription([
        can_bridge_node
    ])