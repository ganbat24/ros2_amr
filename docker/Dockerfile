# amr_ws Dockerfile — ROS 2 Jazzy + Gazebo Harmonic development image
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 40+ node stacks exhaust CycloneDDS' default participant index space
# (surfaces as a gz_ros2_control segfault at the first controller read).
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Gazebo Sim runtime (gz-sim8) ships from the OSRF apt repo; without it
# the image has no `gz` binary and simulation cannot start. Versions are
# pinned to the set verified working on this stack (2026-08).
RUN apt-get update && apt-get install -y wget gnupg lsb-release \
    && wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y gz-harmonic=1.0.0-1~noble libgz-sim8=8.14.0-1~noble \
    && rm -rf /var/lib/apt/lists/*

# System dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    gnupg \
    lsb-release \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-flake8 \
    python3-matplotlib \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-lint \
    ros-jazzy-launch-testing ros-jazzy-launch-testing-ament-cmake \
    ros-jazzy-xacro \
    ros-jazzy-urdf \
    ros-jazzy-urdfdom-py \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
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

# gz_ros2_control 1.2.19 (June 2026 binary) segfaults at the first
# controller read (GazeboSimSystem::read, upstream bug — verified
# 2026-08-10 against the official demo). The jazzy branch carries the
# fix; pin the post-fix tip and build it into the workspace overlay so
# the image works out of the box. Revisit when a fixed binary is out.
# docker/patches/gz_ros2_control-empty-data-guard.patch additionally
# guards the remaining startup race (empty joint-state components under
# load -> Data()[0] null deref).
COPY ./docker/patches /tmp/patches
RUN git init /ros2_ws/src/gz_ros2_control \
    && cd /ros2_ws/src/gz_ros2_control \
    && git remote add origin https://github.com/ros-controls/gz_ros2_control.git \
    && git fetch --depth 1 origin 4047614cfa89d8b77df68d64859d95c3288a3a2e \
    && git checkout --detach FETCH_HEAD \
    && git apply /tmp/patches/gz_ros2_control-empty-data-guard.patch

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update && apt-get update && rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace by default
COPY ./docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# CycloneDDS discovery tuning (see docker/cyclonedds.xml)
COPY ./docker/cyclonedds.xml /etc/cyclonedds.xml

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
