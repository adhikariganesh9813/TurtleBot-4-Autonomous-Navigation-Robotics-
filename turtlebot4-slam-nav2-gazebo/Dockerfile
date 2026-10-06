FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    ros-jazzy-turtlebot4-simulator \
    ros-jazzy-turtlebot4-desktop \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
CMD ["bash"]

