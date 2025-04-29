#!/bin/bash
set -e

# Source ROS and build workspace
source /opt/ros/noetic/setup.bash
cd ${CATKIN_WS}
source devel/setup.bash

# Execute the command passed to docker run
exec "$@"
