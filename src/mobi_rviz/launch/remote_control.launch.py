import launch
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from os import path


def generate_launch_description():
    pkg_share = FindPackageShare(package="mobi_rviz").find("mobi_rviz")
    default_rviz_config_path = path.join(pkg_share, "rviz/mobi.rviz")
    urdf = path.join(pkg_share, "urdf/mobi_4wd.urdf.xacro")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Run rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": ParameterValue(Command(["xacro ", str(urdf)]), value_type=str)}],
        arguments=[urdf],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    rosboard_node = Node(
        package="rosboard",
        executable="rosboard_node"
    )

    # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # teleop_key_node = Node(package="teleop_twist_keyboard", executable="teleop_twist_keyboard", name="teleop_twist_keyboard")

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(name="rvizconfig", default_value=default_rviz_config_path, description="Absolute path to rviz config file"),
            launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"),
            rviz_node,
            robot_state_publisher,
            joint_state_publisher,
            rosboard_node
            # teleop_key_node
        ]
    )
