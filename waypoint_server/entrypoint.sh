#!/bin/bash
set -e

# Source ROS and build workspace
source /opt/ros/noetic/setup.bash
cd ${CATKIN_WS}
source devel/setup.bash

# Wait for ROS master to be available
echo "Waiting for ROS master at $ROS_MASTER_URI..."
until rostopic list > /dev/null 2>&1; do
    sleep 1
done
echo "ROS master is ready!"

source ~/simulation_ws/devel/setup.bash
roslaunch course_web_dev_ros web.launch &

source ~/simulation_ws/devel/setup.bash
roslaunch course_web_dev_ros tf2_web.launch &

source ~/simulation_ws/devel/setup.bash
rosrun course_web_dev_ros tortoisebot_action_server.py

# Execute the command passed to docker run
exec "$@"
