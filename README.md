# ROS Noetic with Docker 
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

## Task1: ROS1 Docker Simulation

### Build and Run Instructions
```bash
# 1. Build all images
cd ~/simulation_ws/src/tortoisebot_ros1_docker
docker-compose build

# 2. Start all services
xhost +local:root  # For GUI applications
docker-compose up -d

# 3. Check running containers
docker-compose ps

# 4. Attach shell to `roscore_gazebo` container
docker-compose exec roscore_gazebo /bin/bash

# 5. Move the robot around using the on-screen joystick present in the Web Application.
# or start/stop using following.
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# 6. Local Links
    ROSBRIGE: ws://localhost:9090
    Webpage: http://localhost:8080
    MJPEG Stream Host: http://localhost:11315

# 7. Stop services
docker-compose down
xhost -local:root
```
