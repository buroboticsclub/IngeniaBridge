import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("arm_trajectory_fanout"),
        "config",
        "arm_trajectory_fanout.yaml",
    )

    return LaunchDescription([
        Node(
            package="arm_trajectory_fanout",
            executable="arm_trajectory_fanout",
            name="arm_trajectory_fanout",
            output="screen",
            parameters=[config],
        )
    ])
