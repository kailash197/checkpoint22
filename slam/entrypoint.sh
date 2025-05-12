#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

GAZEBO_IP=$(getent hosts gazebo_container | awk '{ print $1 }')
SLAM_IP=$(getent hosts slam_container | awk '{ print $1 }')
sed -i "s|<Peer address=\"DOCKER_HOSTNAME\"/>|<Peer address=\"${DOCKER_HOSTNAME}\"/>|g" /home/ttbot/cyclonedds.xml
sed -i "s|<Peer address=\"GAZEBO_CONTAINER\"/>|<Peer address=\"${GAZEBO_IP}\"/>|g" /home/ttbot/cyclonedds.xml
sed -i "s|<Peer address=\"SLAM_CONTAINER\"/>|<Peer address=\"${SLAM_IP}\"/>|g" /home/ttbot/cyclonedds.xml
echo "${DOCKER_HOSTIP} ${DOCKER_HOSTNAME}" | sudo tee -a /etc/hosts
echo "${GAZEBO_IP} gazebo_container" | sudo tee -a /etc/hosts
echo "${SLAM_IP} slam_container" | sudo tee -a /etc/hosts

# source underlay
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/setup.bash

# Replace current shell with the command passed as arguments to ensure proper signal handling
exec "$@"
