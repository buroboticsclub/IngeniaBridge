"""Combined launch: canopen master + ingenia_joint_bridge for j0.

Order:
  1. CANopen master + ProxyDriver for node 10  (drive boots, PDOs configured)
  2. 5 s delay so the master finishes booting before the bridge calls SDO
  3. ingenia_joint_bridge (state machine + IPM streaming)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("ingenia_bringup")
    bridge_share = get_package_share_directory("ingenia_bridge")
    can_interface = LaunchConfiguration("can_interface")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "ingenia_canopen_driver.launch.py")
        ),
        launch_arguments={"can_interface": can_interface}.items(),
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "ingenia_j0_bridge.launch.py")
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="SocketCAN interface name.",
        ),
        driver,
        TimerAction(period=5.0, actions=[bridge]),
    ])
