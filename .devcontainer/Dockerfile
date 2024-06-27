ARG ROS_DISTRO=humble

FROM althack/ros2:$ROS_DISTRO-full 

# ** USER ROOT **
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
        nano \
        libserial-dev \
        ros-$ROS_DISTRO-teleop-twist-keyboard \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-rviz-common \
        ros-$ROS_DISTRO-rviz-default-plugins \
        ros-$ROS_DISTRO-rviz-visual-tools \
        ros-$ROS_DISTRO-rviz-rendering \
        ros-$ROS_DISTRO-nav2-rviz-plugins \
        ros-$ROS_DISTRO-navigation2 \ 
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-slam-toolbox \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
        ros-$ROS_DISTRO-urdf-launch \
        ros-$ROS_DISTRO-robot-localization \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*


RUN usermod -aG dialout,video ros

# ** USER ROS **
USER ros
ENV USER=ros

RUN mkdir -p ~/.ssh && ssh-keyscan -T 10 github.com >> ~/.ssh/known_hosts

# setup dotfiles
# RUN --mount=type=ssh,required=true,mode=0666 git clone git@github.com:Flo2410/dotfiles.git --recurse-submodules ~/dotfiles && \
RUN sudo apt-get update && \
    git clone https://github.com/Flo2410/dotfiles.git --recurse-submodules ~/dotfiles && \
    cd ~/dotfiles && \
    ./install-profile mjölnir \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

RUN echo export RMW_IMPLEMENTATION=rmw_fastrtps_cpp >> ~/.zshrc && \
    echo source /opt/ros/$ROS_DISTRO/setup.zsh >> ~/.zshrc

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
WORKDIR /workspaces
