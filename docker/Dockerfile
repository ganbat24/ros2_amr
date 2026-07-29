# amr_ws Dockerfile — ROS 2 Jazzy + Gazebo Harmonic development image
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# System dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-flake8 \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-lint \
    ros-jazzy-launch-testing ros-jazzy-launch-testing-ament-cmake \
    ros-jazzy-xacro \
    ros-jazzy-urdf \
    ros-jazzy-urdfdom-py \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-gz-ros2-control \
     \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-nav-msgs \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-dwb-controller \
     \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-amcl \
    ros-jazzy-image-proc \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*

# Pre-compile setup
COPY ./src /ros2_ws/src
WORKDIR /ros2_ws
RUN rosdep update && apt-get update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace by default
COPY ./docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
