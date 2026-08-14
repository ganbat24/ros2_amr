# amr_ws Dockerfile — ROS 2 Jazzy + Gazebo Harmonic development image
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
# RMW: the ROS 2 Jazzy default (rmw_fastrtps_cpp) — deliberately not
# overridden. This image previously forced rmw_cyclonedds_cpp plus a tuned
# /etc/cyclonedds.xml that raised MaxAutoParticipantIndex, because stale
# participants from killed launches exhausted Cyclone's auto index space and
# surfaced as a gz_ros2_control nil-deref at the first controller read.
#
# That was a Cyclone-specific leak in a workspace whose validation harness
# kills launches as a routine recovery step, and it made the container the
# only environment running that middleware — WSL2, where the simulation
# actually runs, has always used the FastRTPS default. Standardising on the
# default removes the failure mode instead of tuning around it, and makes the
# container, CI and WSL2 genuinely identical.
#
# FastRTPS has its own cleanup obligation: a SIGKILLed participant leaves
# shared-memory segments behind, and a later run can fail to bind. Clear them
# with `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` after a hard kill
# (amr_metrics' relaunch path already does this).

# Gazebo comes from ros-jazzy-ros-gz's vendored build (gz_sim_vendor), pulled
# in with the ROS packages below — NOT from the OSRF apt repo.
#
# This image used to also install gz-harmonic=1.0.0-1~noble and
# libgz-sim8=8.14.0-1~noble from OSRF, "pinned to the set verified working".
# That pin never bound: sourcing ROS puts /opt/ros/jazzy/opt/gz_*_vendor first
# on LD_LIBRARY_PATH and GZ_CONFIG_PATH, including the `gz` binary itself, and
# every ros2 launch runs ROS-sourced. The image therefore carried two complete
# Gazebo installations (apt 8.14.0, vendored 8.11.0) and simulated on the
# vendored one while the Dockerfile advertised the other — so `gz sim
# --versions` answered differently depending on whether you had sourced ROS.
#
# CI has never installed the OSRF packages and works, which is the proof that
# they were redundant. Dropping them removes the ambiguity, shrinks the image,
# and makes the container match CI exactly. Check the version that matters
# from inside a ROS-sourced shell.

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

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
