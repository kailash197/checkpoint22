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

# Task2: ROS2 Tortoisebot Simulation in Gazebo

## Run Instructions

### 1. Pull images from dockerhub to docker host
Command to manually pull these images from dockerhub:
Terminal [PC]:
```bash
docker pull kkhadka343/kailash-cp22:tortoisebot-ros2-gazebo
docker pull kkhadka343/kailash-cp22:tortoisebot-ros2-slam
```
### 2. Clone the repo
Terminal [PC]:
```bash
mkdir -p ~/ros2_ws/src # if doesn't exist yet
cd ~/ros2_ws/src
git clone -b ros2-galactic https://github.com/kailash197/checkpoint22.git
mv checkpoint22 tortoisebot_ros2_docker
cd ~/ros2_ws/src/tortoisebot_ros2_docker
```

### 3. Start all services
Terminal [PC]:
```bash
xhost +local:root  # For GUI applications

cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose up -d
```
All the services including tortoisebot simulation in gazebo and slam container are automatically started by the end of this step. 
Docker service `gazebo` running in `gazebo_container` includes:
- Tortoisebot simulation in gazebo
- Lidar node
- Camera node

Docker service `slam` running in `slam_container` includes:
- Cartographer node
- Rviz2 node
- Teleop [See step 3]

#### Check images & running containers
- Check images  
    Terminal [PC]:
    ```bash
    docker images
    ```
    Expected Output:
    ```bash
    REPOSITORY                TAG                       IMAGE ID       CREATED        SIZE
    kkhadka343/kailash-cp22   tortoisebot-ros2-slam     21cd5333026c   8 hours ago    3.75GB
    tortoisebot-ros2-slam     v1                        21cd5333026c   8 hours ago    3.75GB
    kkhadka343/kailash-cp22   tortoisebot-ros2-gazebo   90b5d12259b1   8 hours ago    3.55GB
    tortoisebot-ros2-gazebo   v1                        90b5d12259b1   8 hours ago    3.55GB
    ```

- Check containers  
    Terminal [PC]:
    ```bash
    docker-compose ps
    ```
    Output:
    ```bash
        Name                    Command               State   Ports
    -----------------------------------------------------------------
    gazebo_container   /entrypoint.sh /bin/bash - ...   Up
    slam_container     /entrypoint.sh /bin/bash - ...   Up
    ```
    
    ```bash
    docker ps
    ```
    Output:
    ```bash
    CONTAINER ID   IMAGE                           COMMAND                  CREATED              STATUS              PORTS     NAMES
    54907b3b4795   tortoisebot-ros2-slam:v1        "/entrypoint.sh /bin…"   About a minute ago   Up About a minute             slam_container
    5aacd1a002f5   tortoisebot-ros2-gazebo:v1      "/entrypoint.sh /bin…"   About a minute ago   Up About a minute             gazebo_container
    ```

### 4. Move the tortoisebot
Run `telop` program in `slam_container` (`slam` service) to move the tortoisebot around.  
Terminal [PC]:
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose exec slam bash -c \
    "source /opt/ros/galactic/setup.bash \
    && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

### 5. Stop services
Terminal [PC]:
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose down
xhost -local:root
```
