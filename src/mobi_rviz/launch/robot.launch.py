import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="mobi_rviz").find("mobi_rviz")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/rplidar.rviz")

    lidar_node = launch_ros.actions.Node(
        name="rplidar_composition",
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[
            {
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
            }
        ],
    )

    mobictl = launch_ros.actions.Node(
        package="mobictl_cpp",
        executable="mobictl",
        name="mobictl",
        parameters=["port", "/dev/ttyACM0"]
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(name="rvizconfig", default_value=default_rviz_config_path, description="Absolute path to rviz config file"),
            lidar_node,
            mobictl
        ]
    )
