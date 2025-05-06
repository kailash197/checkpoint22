#!/bin/bash
set -e

# Source ROS and build workspace
source /opt/ros/noetic/setup.bash
cd ${CATKIN_WS}

source /home/ttbot/ros1_ws/devel/setup.bash
roslaunch tortoisebot_firmware bringup.launch

# Execute the command passed to docker run
exec "$@"
