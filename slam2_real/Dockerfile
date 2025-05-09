# ===== ARM64-Compatible Base Image =====
FROM kkhadka343/kailash-cp22:tortoisebot-ros2-real

# ===== Timezone Configuration =====
ARG timezone=UTC
ENV TZ=${timezone}

# ===== User Setup =====
USER root
ARG USERNAME=ttbot
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS2_WS=/home/${USERNAME}/ros2_ws
SHELL ["/bin/bash", "-c"]

# ===== ROS 2 Dependencies =====
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-galactic-cartographer \
        ros-galactic-cartographer-ros \
        # ros-galactic-gazebo-plugins \
        # ros-galactic-nav2-bringup \
        # ros-galactic-nav2-map-server \
        # ros-galactic-nav2-amcl \
        # ros-galactic-nav2-planner \
        # ros-galactic-nav2-controller \
        # ros-galactic-nav2-behavior-tree \
        # ros-galactic-nav2-lifecycle-manager \
        # ros-galactic-nav2-waypoint-follower \
        # ros-galactic-nav2-navfn-planner \
        # ros-galactic-nav2-costmap-2d \
        # ros-galactic-nav2-regulated-pure-pursuit-controller \
    && rm -rf /var/lib/apt/lists/*

# ===== Workspace Setup =====
USER ${USERNAME}
WORKDIR ${ROS2_WS}

# ===== Copy and Build =====
COPY --chown=${USER_UID}:${USER_GID} ./tortoisebot src/tortoisebot
COPY --chown=${USER_UID}:${USER_GID} ./raspicam2_node src/raspicam2_node

# ===== Build =====
RUN . /opt/ros/galactic/setup.bash \
    && colcon build --symlink-install \
    && echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc \
    && echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# ===== Networking Configuration =====
ENV ROS_DISTRO=galactic
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=7
COPY --chown=${USER_UID}:${USER_GID} ./tortoisebot_ros2_docker/ros2_real/cyclonedds.xml /home/${USERNAME}/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///home/${USERNAME}/cyclonedds.xml

# ===== Entrypoint =====
COPY --chown=${USER_UID}:${USER_GID} ./tortoisebot_ros2_docker/ros2_real/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
