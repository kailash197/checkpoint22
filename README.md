# ROS2 Galactic with Docker 
For this Checkpoint, I had to create several Docker images in order to manage the bringup of the TortoiseBot robot, for both the simulation and the real robot. Containerizing the robot applications simplifies the whole process of setting up and starting to use a robot, especially for new users who are not used to working with robots.

## Prerequisites

- Linux system (Ubuntu recommended)
- X11 server running (for GUI applications)
- Basic familiarity with terminal commands

## Installation Steps

### 1. Install Docker and Docker Compose

```bash
# Check if Docker is installed
# returns path to docker if installed already
command -v docker

# If not installed:
sudo apt-get update
sudo apt-get install -y docker.io docker-compose

# Start Docker
sudo systemctl enable docker
sudo systemctl start docker

# To use Docker without sudo
sudo usermod -aG docker $USER
newgrp docker
```

### 2. X11 Server Setup (for GUI Applications) 
```bash
# Check if X11 Server is installed
command -v xhost

# If not installed:
sudo apt-get update
sudo apt-get install -y x11-xserver-utils
```

## Task2: ROS2 Tortoisebot Simulation in Gazebo

### Build and Run Instructions
#### 1. Build all images (uncomment build section in `docker-compose.yml`)
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose build
```

#### 2. Start all services
```bash
xhost +local:root  # For GUI applications
docker-compose up -d
```
All the services including tortoisebot simulation in gazebo and SLAM container are automatically started by the end of this step.

#### 3. Check images & running containers
- Check images
    ```bash
    docker images
    ```
    Expected Output:
    ```bash
    REPOSITORY                TAG                       IMAGE ID       CREATED             SIZE
    tortoisebot-ros2-slam     v1                        3bc3e31ead0c   13 minutes ago      3.84GB
    tortoisebot-ros2-gazebo   v1                        328555225386   42 minutes ago      3.54GB
    osrf/ros                  galactic-desktop          57a31ebe075c   16 months ago       3.05GB
    ```
- Check containers
    ```bash
    docker-compose ps
    ```
    Expected Output:
    ```bash
        Name                    Command               State   Ports
    -----------------------------------------------------------------
    gazebo_container   /entrypoint.sh /bin/bash - ...   Up
    slam_container     /entrypoint.sh /bin/bash - ...   Up
    ```

#### 4. Move the tortoisebot
Run `telop` program in `slam_container` (`slam` service) to move the tortoisebot around.
```bash
docker-compose exec slam bash -c \
    "source /opt/ros/galactic/setup.bash \
    && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

#### 6. Stop services
```bash
docker-compose down
xhost -local:root
```
