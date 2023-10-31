from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare("mobi_rviz")
    joy_params = PathJoinSubstitution([pkg_share, "config/joystick.yaml"])

    # Lidar
    lidar_node = Node(
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

    # mobictl
    mobictl = Node(package="mobictl_cpp", executable="mobictl", name="mobictl", parameters=["port", "/dev/ttyACM0"])

    # gamepad
    joy_node = Node(package="joy", executable="joy_node", parameters=[joy_params])

    # telepo (/joy -> /cmd_vel)
    teleop_node = Node(package="teleop_twist_joy", executable="teleop_node", name="teleop_node", parameters=[joy_params])

    return LaunchDescription(
        [
            lidar_node,
            mobictl,
            joy_node,
            teleop_node,
        ]
    )
