#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# source underlay
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/setup.bash

# Replace current shell with the command passed as arguments to ensure proper signal handling
exec "$@"
