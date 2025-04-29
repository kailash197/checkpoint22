FROM osrf/ros:noetic-desktop-full

# ===== User Setup =====
ARG USERNAME=ttbot
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS=/home/${USERNAME}/simulation_ws
SHELL ["/bin/bash", "-c"]

# ===== System Dependencies =====
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME} \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        curl \
        python3-rosdep \
        python3-catkin-tools \
        python3-pip \
        python3-lxml \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# ===== Python Configuration =====
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# ===== ROS/Gazebo Dependencies =====
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-noetic-rosbridge-server \
        ros-noetic-web-video-server \
        ros-noetic-tf2-web-republisher \
        ros-noetic-rosauth \
        ros-noetic-robot-state-publisher \
        ros-noetic-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# ===== User Context + Workspace Setup =====
USER ${USERNAME}
WORKDIR ${CATKIN_WS}

# ===== Copy and Build =====
RUN mkdir src
COPY --chown=${USER_UID}:${USER_GID} ./course_web_dev_ros ${CATKIN_WS}/src
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make \
    && source devel/setup.bash \
    && echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source ${CATKIN_WS}/devel/setup.bash" >> /home/${USERNAME}/.bashrc

# ===== Environment Variables =====
# Persistent ROS settings (avoids needing `source` in every shell)
ENV ROS_DISTRO=noetic
ENV ROS_MASTER_URI=http://gazebo_container:11311
ENV ROS_HOSTNAME=waypoint_server_container
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# ===== Entrypoint =====
COPY --chown=${USER_UID}:${USER_GID} ./tortoisebot_ros1_docker/waypoint_server/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]