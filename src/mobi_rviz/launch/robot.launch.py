from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare("mobi_rviz")
    joy_params = PathJoinSubstitution([pkg_share, "config/joystick.yaml"])
    lidar_params = PathJoinSubstitution([pkg_share, "config/lidar.yaml"])

    # Lidar
    lidar_node = Node(
        name="rplidar",
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[lidar_params],
    )

    # gamepad
    joy_node = Node(package="joy", executable="joy_node", parameters=[joy_params])

    # telepo (/joy -> /cmd_vel)
    teleop_node = Node(package="teleop_twist_joy", executable="teleop_node", name="teleop_node", parameters=[joy_params])

    return LaunchDescription(
        [
            lidar_node,
            joy_node,
            teleop_node,
        ]
    )
