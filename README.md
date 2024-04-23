# MobiRviz

This is a demo application for the "Mobi" mobile robot.

# Install Infos

Here are some install infos. These are not (yet) a complet install guide.

## Generating the `micro-ROS agent`

1. Build the workspace

    ```bash
    rosdep update && rosdep install --from-paths src --ignore-src -y
    colcon build
    source install/local_setup.bash
    ```

2. Create and build the agent

    ```bash
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.sh
    ```