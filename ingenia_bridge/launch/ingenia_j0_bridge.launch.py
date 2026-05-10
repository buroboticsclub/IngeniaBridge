"""Launch the Ingenia bridge for joint 0 (EVS-XCR-C, CAN node 10)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("ingenia_bridge"),
        "config",
        "ingenia_j0.yaml",
    )

    return LaunchDescription([
        Node(
            package="ingenia_bridge",
            executable="ingenia_joint_bridge",
            name="ingenia_j0_bridge",
            output="screen",
            parameters=[config],
            remappings=[
                ("fault", "/ingenia/j0/fault"),
                ("reduced_traj", "/ingenia/j0/reduced_traj"),
                ("clear_fault", "/ingenia/j0/clear_fault"),
                ("arm_ipm", "/ingenia/j0/arm_ipm"),
                ("disarm_ipm", "/ingenia/j0/disarm_ipm"),
                ("move_absolute_timed", "/ingenia/j0/move_absolute_timed"),
            ],
        )
    ])
