import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="mobi_rviz").find("mobi_rviz")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/rplidar.rviz")

    # Run rviz2
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # teleop_key_node = launch_ros.actions.Node(package="teleop_twist_keyboard", executable="teleop_twist_keyboard", name="teleop_twist_keyboard")

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(name="rvizconfig", default_value=default_rviz_config_path, description="Absolute path to rviz config file"),
            rviz_node,
            # teleop_key_node
        ]
    )
