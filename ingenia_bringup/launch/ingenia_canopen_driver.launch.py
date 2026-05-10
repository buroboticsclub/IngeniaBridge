"""CANopen master + ProxyDriver for Ingenia EVS-XCR-C at node 10.

After this comes up, the drive is in Operational state with the bridge's
expected PDO mapping, and /node_10/sdo_read + /node_10/sdo_write services
are available for ingenia_joint_bridge to use.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("ingenia_bringup")
    can_interface = LaunchConfiguration("can_interface")

    canopen = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("canopen_core"),
                         "launch", "canopen.launch.py")
        ),
        launch_arguments={
            "master_config": os.path.join(bringup_share, "config", "j0", "master.dcf"),
            "master_bin": "",
            "bus_config": os.path.join(bringup_share, "config", "j0", "bus.yml"),
            "can_interface_name": can_interface,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="SocketCAN interface name.",
        ),
        canopen,
    ])
