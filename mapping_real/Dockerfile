FROM kkhadka343/kailash-cp22:tortoisebot-ros1-real

# ===== Root Section =====
USER root
ARG USERNAME=ttbot
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
ARG USERNAME=ttbot
ENV CATKIN_WS=/home/${USERNAME}/ros1_ws

# Install ROS packages
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-noetic-joy \
        ros-noetic-teleop-twist-joy \
        ros-noetic-amcl \
        ros-noetic-map-server \
        ros-noetic-move-base \
        ros-noetic-rqt-image-view \
        ros-noetic-gmapping \
        ros-noetic-navigation \
        ros-noetic-slam-gmapping \
        ros-noetic-dwa-local-planner \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-ira-laser-tools \ 
        python3-wstool \
        python3-sphinx \
        ninja-build \
        stow \
    && sudo rm -rf /var/lib/apt/lists/*

# Create and build cartographer workspace
RUN mkdir -p ${CATKIN_WS}/carto_ws \
    && cd ${CATKIN_WS}/carto_ws \
    && wstool init src \
    && wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall \
    && wstool update -t src

RUN sed -i 's/ <depend>libabsl-dev<\/depend>/<!-- depend>libabsl-dev<\/depend-->/' ${CATKIN_WS}/carto_ws/src/cartographer/package.xml

# ===== ROS/Gazebo Dependencies =====
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        gazebo11 \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

RUN sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
    && (sudo rosdep init || true) \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=noetic -y

WORKDIR ${CATKIN_WS}/carto_ws/src

# Clone additional perception packages
RUN git clone -b melodic-devel https://github.com/ros-perception/perception_pcl.git \
    && git clone https://github.com/ros-perception/pcl_msgs \
    && git clone -b noetic-devel https://github.com/jsk-ros-pkg/geometry2_python3.git

WORKDIR ${CATKIN_WS}
RUN source /opt/ros/noetic/setup.bash && catkin_make && source devel/setup.bash

WORKDIR ${CATKIN_WS}/carto_ws
RUN apt-get update \
    && apt-get install -y sudo lsb-release curl gnupg \
    && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && apt-get update \
    && apt-get install -y libgflags-dev

RUN source /opt/ros/noetic/setup.bash \
    && source ${CATKIN_WS}/devel/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro=noetic -y \
    && src/cartographer/scripts/install_abseil.sh

RUN source /opt/ros/noetic/setup.bash \
    && source ${CATKIN_WS}/devel/setup.bash \
    && catkin_make_isolated --install --use-ninja -j3 -l3 \
    && source devel_isolated/setup.bash 

RUN source /opt/ros/noetic/setup.bash \
    && source ${CATKIN_WS}/carto_ws/devel_isolated/setup.bash \
    && cd ${CATKIN_WS} \
    && rm -rf ./build ./devel \
    && catkin_make \
    && source devel/setup.bash

# Set up environment
RUN echo "source ${CATKIN_WS}/carto_ws/install_isolated/setup.bash" >> /home/${USERNAME}/.bashrc
RUN chmod +x ${CATKIN_WS}/src/tortoisebot_firmware/scripts/odom_pub.py
WORKDIR ${CATKIN_WS}

# ===== Environment Variables =====
ENV ROS_DISTRO=noetic
ENV ROS_MASTER_URI=http://roscore_real_container:11311
ENV ROS_HOSTNAME=mapping_real_container
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# ===== Entrypoint =====
USER ${USERNAME}
COPY --chown=${USER_UID}:${USER_GID} ./tortoisebot_ros1_docker/mapping_real/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
