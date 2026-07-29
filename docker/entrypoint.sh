#!/bin/bash
set -e

# Source ROS 2 base
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace overlay if it exists
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Ensure X11 auth file exists (Docker creates it as directory if missing)
if [ -n "${DISPLAY}" ] && [ ! -f "${XAUTHORITY:-/tmp/.docker.xauth}" ]; then
    touch "${XAUTHORITY:-/tmp/.docker.xauth}"
fi

exec "$@"
