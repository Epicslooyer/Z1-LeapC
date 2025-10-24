#!/bin/bash
set -e

if [ -n "${ROS_DISTRO}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ -n "${CATKIN_WS}" ] && [ -f "${CATKIN_WS}/devel/setup.bash" ]; then
  source "${CATKIN_WS}/devel/setup.bash"
fi

export LEAP_SDK_ROOT="${LEAP_SDK_ROOT:-/opt/leap-sdk}"
export LD_LIBRARY_PATH="${LEAP_SDK_ROOT}/lib:${LD_LIBRARY_PATH}"
export LIBRARY_PATH="${LEAP_SDK_ROOT}/lib:${LIBRARY_PATH}"
export CMAKE_PREFIX_PATH="${LEAP_SDK_ROOT}:${CMAKE_PREFIX_PATH}"

exec "$@"
