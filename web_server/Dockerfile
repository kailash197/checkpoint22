FROM ubuntu:20.04

# ===== User Setup =====
ARG USERNAME=ttbot
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV WEBPAGE_WS=/home/${USERNAME}/webpage_ws
ENV DEBIAN_FRONTEND=noninteractive

# ===== System Dependencies =====
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    nginx \
    && rm -rf /var/lib/apt/lists/*

# # ===== Configure Nginx =====
# COPY ./tortoisebot_ros1_docker/web_server/nginx.conf /etc/nginx/conf.d/default.conf
# RUN ln -sf /etc/nginx/sites-available/default /etc/nginx/sites-enabled/ && \
#     rm -f /etc/nginx/sites-enabled/default

# ===== User Setup =====
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    chown -R ${USER_UID}:${USER_GID} /var/www/html

# ===== Web Content =====
WORKDIR ${WEBPAGE_WS}
COPY --chown=${USER_UID}:${USER_GID} ./webpage_4_robots /var/www/html/
RUN sed -i 's/80\ default_server/8001\ default_server/' /etc/nginx/sites-enabled/default

# ===== ROS Environment =====
ENV ROS_DISTRO=noetic
ENV ROS_MASTER_URI=http://gazebo_container:11311
ENV ROS_HOSTNAME=webpage_container

# ===== Entrypoint =====
USER root
COPY --chown=${USER_UID}:${USER_GID} ./tortoisebot_ros1_docker/web_server/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
