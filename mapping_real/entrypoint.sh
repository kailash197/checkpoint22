#!/bin/bash
set -e

# Source ROS and build workspace
source /opt/ros/noetic/setup.bash

# Wait for ROS master to be available
echo "Waiting for ROS master at $ROS_MASTER_URI..."
until rostopic list > /dev/null 2>&1; do
    sleep 1
done
echo "ROS master is ready!"

cd ${CATKIN_WS}
source ${CATKIN_WS}/devel/setup.bash

source ${CATKIN_WS}/carto_ws/devel_isolated/setup.bash
source ${CATKIN_WS}/devel/setup.bash

echo "$(date +'[%Y-%m-%d %T]') Starting server bringup..."
roslaunch tortoisebot_firmware server_bringup.launch &

echo "$(date +'[%Y-%m-%d %T]') Starting slam..."
roslaunch tortoisebot_slam tortoisebot_slam.launch

# Execute the command passed to docker run
exec "$@"