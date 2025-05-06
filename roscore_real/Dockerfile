FROM  ros:noetic-ros-core-focal

# ===== User Setup =====
ARG USERNAME=ttbot
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS=/home/${USERNAME}/ros1_ws
SHELL ["/bin/bash", "-c"]

# ===== System Dependencies =====
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir -p ${CATKIN_WS}/src \
    && chown -R $USER_UID:$USER_GID /home/${USERNAME} \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        curl \
        apt-utils \
        sudo \
        python3-rosdep \
        python3-catkin-tools \
        python3-pip \
        python3-lxml \
        build-essential \
        cmake \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# ===== Python Configuration =====
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# ===== REAL ROBOT Dependencies (No Gazebo) =====
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-hardware-interface \
    ros-noetic-controller-manager \
    ros-noetic-camera-info-manager \
    ros-noetic-diagnostic-updater \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-compressed-image-transport \
    ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libraspberrypi-dev \
        libraspberrypi0 \
        python3-rpi.gpio \
        libraspberrypi-bin \
    && usermod -a -G dialout ${USERNAME} \
    && rm -rf /var/lib/apt/lists/*
    # libmmal-dev \ unable to locate package


RUN mkdir -p /etc/ros/rosdep/sources.list.d \
    && echo "yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml" > /etc/ros/rosdep/sources.list.d/30-ubiquity.list \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep install --from-paths /home/${USERNAME}/ros1_ws/src --ignore-src -r -y

# Create video group and add user
RUN usermod -aG video ${USERNAME}

# Create symlink directory
RUN mkdir -p /usr/lib

# ===== User Context + Workspace Setup =====
USER ${USERNAME}
WORKDIR ${CATKIN_WS}

# ===== Copy and Build =====
COPY --chown=${USER_UID}:${USER_GID} ./tortoisebot ${CATKIN_WS}/src/tortoisebot
COPY --chown=${USER_UID}:${USER_GID} ./raspicam_node ${CATKIN_WS}/src/raspicam_node

RUN source /opt/ros/noetic/setup.bash \
    && catkin_make \
    && source devel/setup.bash \
    && echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source ${CATKIN_WS}/devel/setup.bash" >> /home/${USERNAME}/.bashrc

# ===== Environment Variables =====
ENV ROS_DISTRO=noetic
ENV ROS_MASTER_URI=http://roscore_real_container:11311
ENV ROS_HOSTNAME=roscore_real_container
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# ===== Entrypoint =====
COPY --chown=${USER_UID}:${USER_GID} ./tortoisebot_ros1_docker/roscore_real/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]