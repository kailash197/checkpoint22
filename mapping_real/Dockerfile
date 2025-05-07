FROM ros:noetic-robot

# ===== Root Section =====
USER root
ARG USERNAME=ttbot
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
ENV CATKIN_WS=/home/${USERNAME}/ros1_ws

# Install system dependencies first
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    ninja-build \
    stow \
    python3-wstool \
    python3-rosdep \
    python3-sphinx \
    libgflags-dev \
    libgoogle-glog-dev \
    libceres-dev \
    liblua5.3-dev \
    && rm -rf /var/lib/apt/lists/*

# Create Lua symlink for Cartographer compatibility
RUN ln -s /usr/lib/$(uname -m)-linux-gnu/liblua5.3.so /usr/lib/$(uname -m)-linux-gnu/liblua5.2.so || \
    echo "Warning: Lua symlink creation failed - proceeding anyway"

# Create and build cartographer workspace
RUN mkdir -p ${CATKIN_WS}/carto_ws/src \
    && cd ${CATKIN_WS}/carto_ws \
    && wstool init src \
    && wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
    && wstool update -t src

# Patch package.xml before building
RUN sed -i 's/<depend>libabsl-dev<\/depend>/<!-- depend>libabsl-dev<\/depend -->/' \
    ${CATKIN_WS}/carto_ws/src/cartographer/package.xml

# Add universe repository first
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && apt-get update

# Then run rosdep install
RUN rosdep install --from-paths ${CATKIN_WS}/carto_ws/src --ignore-src --rosdistro=noetic -y \
    --skip-keys="libprotobuf-dev"

# Initialize rosdep and install dependencies
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
    && (rosdep init || true) \
    && rosdep update \
    && rosdep install --from-paths ${CATKIN_WS}/carto_ws/src --ignore-src --rosdistro=noetic -y

# Install Abseil manually
RUN cd ${CATKIN_WS}/carto_ws \
    && src/cartographer/scripts/install_abseil.sh

# Build Cartographer with isolated build
WORKDIR ${CATKIN_WS}/carto_ws
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make_isolated --install --use-ninja \
    && source install_isolated/setup.bash

# ======= Create User =======
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir -p ${CATKIN_WS}/src \
    && chown -R $USER_UID:$USER_GID /home/${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && usermod -a -G dialout ${USERNAME} \
    && usermod -aG video ${USERNAME}

# ===== Copy and Build =====
COPY ./tortoisebot ${CATKIN_WS}/src/tortoisebot
COPY ./raspicam_node ${CATKIN_WS}/src/raspicam_node

RUN apt-get update \
    && apt-get install -y \
    ros-noetic-compressed-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-image-transport \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-nodelet \
    ros-noetic-roscpp \
    ros-noetic-sensor-msgs \
    libraspberrypi-dev \
    libraspberrypi0 \
    python3-rpi.gpio \
    libraspberrypi-bin \
    && rm -rf /var/lib/apt/lists/*

# ===== User Context + Workspace Setup =====
USER ${USERNAME}
WORKDIR ${CATKIN_WS}

# Build main workspace
RUN source /opt/ros/noetic/setup.bash \
    && source ${CATKIN_WS}/carto_ws/install_isolated/setup.bash \
    && cd ${CATKIN_WS} \
    && rm -rf ./build ./devel \
    && catkin_make \
    && source devel/setup.bash

# Set up environment
RUN echo "source ${CATKIN_WS}/carto_ws/install_isolated/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source ${CATKIN_WS}/devel/setup.bash" >> /home/${USERNAME}/.bashrc

# ===== Environment Variables =====
ENV ROS_DISTRO=noetic
ENV ROS_MASTER_URI=http://roscore_real_container:11311
ENV ROS_HOSTNAME=mapping_real_container
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# ===== Entrypoint =====
WORKDIR ${CATKIN_WS}
COPY ./tortoisebot_ros1_docker/mapping_real/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
