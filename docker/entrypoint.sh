#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${COLCON_WS}/install/setup.bash

exec "$@"
