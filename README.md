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
#### 1. Optional: Build all images (uncomment build section in `docker-compose.yml`)
```bash
cd ~/simulation_ws/src/tortoisebot_ros1_docker
docker-compose build
```

#### 2. Start all services
```bash
xhost +local:root  # For GUI applications
docker-compose up -d
```
All the services including roscore, tortoisebot simulation in gazebo, waypoints action server, rosbridge server, web video server, mapping container are automatically started by the end of this step.

#### 3. Check running containers
```bash
docker-compose ps
```

#### 4. Running Web Appliation
Nginx Webserver should be up and running by the end of step 2.
In this step, find the webpage address by running following command and open it on any web browser.
```bash
webpage_address

# Sample address:
# https://i-014c91ec77897b767.robotigniteacademy.com/a7e86461-5712-4eb2-bb09-a7344a6f5eb9/webpage/
```
Following command returns the rosbridge address. Enter this address on the webpage and click connect.
```bash
rosbridge_address

# Sample address
# wss://i-014c91ec77897b767.robotigniteacademy.com/a7e86461-5712-4eb2-bb09-a7344a6f5eb9/rosbridge/
```
MJPEG Stream Host: http://localhost:11315

#### 5. Move the tortoisebot
Use the on-screen joystick present in the Web Application to move the tortoisebot around.

#### 6. Stop services
```bash
docker-compose down
xhost -local:root
```

### Useful commands
1. Attach shell to `roscore_gazebo` container and test connection to the robot.
```bash
docker-compose exec roscore_gazebo /bin/bash

rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```
