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

### Run Instructions

#### 1. If necessary, pull images from dockerhub to PC
Terminal`[PC]`
```bash
docker pull kkhadka343/kailash-cp22:tortoisebot-ros1-gazebo
docker pull kkhadka343/kailash-cp22:tortoisebot-ros1-slam
docker pull kkhadka343/kailash-cp22:tortoisebot-ros1-waypoints
docker pull kkhadka343/kailash-cp22:tortoisebot-ros1-webapp
```

#### 2. Start all services
Go to the required directory and start services.
Terminal`[PC]`
```bash
xhost +local:root  # For GUI applications

cd ~/simulation_ws/src/tortoisebot_ros1_docker
docker-compose up -d
```

All the services including roscore, tortoisebot simulation in gazebo, waypoints action server, rosbridge server, web video server, mapping container are automatically started by the end of this step.

#### 3. Check running containers
Terminal`[PC]`
```bash
docker-compose ps
```

#### 4. Running Web Appliation
Nginx Webserver should be up and running by the end of step 2.
In this step, find the webpage address by running following command and open it on any web browser.
Terminal`[PC]`
```bash
webpage_address

# Sample address:
# https://i-014c91ec77897b767.robotigniteacademy.com/a7e86461-5712-4eb2-bb09-a7344a6f5eb9/webpage/
```
Following command returns the rosbridge address. Enter this address on the webpage and click connect.
Terminal`[PC]`
```bash
rosbridge_address

# Sample address
# wss://i-014c91ec77897b767.robotigniteacademy.com/a7e86461-5712-4eb2-bb09-a7344a6f5eb9/rosbridge/
```
MJPEG Stream Host: http://localhost:11315

#### 5. Move the tortoisebot
Use the on-screen joystick present in the Web Application to move the tortoisebot around.

#### 6. Stop services
Terminal`[PC]`
```bash
cd ~/simulation_ws/src/tortoisebot_ros1_docker
docker-compose down
xhost -local:root
```

### Useful commands
1. Attach shell to `roscore_gazebo` container.
Terminal`[PC]`
```bash
docker-compose exec roscore_gazebo /bin/bash
```

2. Send test velocites to the robot.
Terminal`[PC]`
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Task3: ROS1 Docker with Real Robot

#### 1. Pull images from dockerhub to PC
Terminal`[Raspberry]`
```bash
docker pull kkhadka343/kailash-cp22:tortoisebot-ros1-real
docker pull kkhadka343/kailash-cp22:tortoisebot-ros1-slam-real
```

#### 2. Start all services

Bring up all services using following command. Notice the docker compose file for real robot.
Terminal `[Raspberry]`
```bash
cd ~/ros1_ws/src/tortoisebot_ros1_docker
docker-compose -f docker-compose.real.yml up -d
```

#### 3. Check images and  running containers
- Check images  
    Terminal `[Raspberry]`
    ```bash
    docker images
    ```

    Sample output:
    ```bash
    REPOSITORY                TAG                     IMAGE ID       CREATED          SIZE
    kkhadka343/kailash-cp22   tortoisebot-ros1-real   b14d2f0446d0   31 minutes ago   3.01GB
    tortoisebot-ros1-real     v1                      b14d2f0446d0   31 minutes ago   3.01GB
    <none>                    <none>                  7ea4fb6c4838   3 hours ago      2.92GB
    ros                       noetic-ros-core-focal   49ac5c2cfabd   2 months ago     740MB
    ```

- Check containers
    Terminal `[Raspberry]`
    ```bash
    cd ~/ros1_ws/src/tortoisebot_ros1_docker
    docker-compose -f docker-compose.real.yml ps
    ```

    Sample Output:
    ```bash        
            Name               Command       State                      Ports
    ----------------------------------------------------------------------------------------------
    mapping_real_container   /entrypoint.sh   Up
    roscore_real_container   /entrypoint.sh   Up      0.0.0.0:11311->11311/tcp,:::11311->11311/tcp
    ```
#### 4. Start RVIZ for visualization
Start RVIZ in PC. Avoid starting RVIZ in the robot which has limited resources.
Terminal `[PC]`
```bash
source /opt/ros/noetic/setup.bash
source ~/ros1_ws/devel/setup.bash
rviz -d ./src/tortoisebot/tortoisebot_slam/rviz/mapping.rviz
```

#### 5. Move the tortoisebot
Run telop program in `roscore_real` service to move the tortoisebot around.
Terminal `[Raspberry]`
```bash
docker-compose -f docker-compose.real.yml exec roscore_real bash -c \
    "source /home/ttbot/ros1_ws/devel/setup.bash \
    && rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
```

#### 6. Stop services
Terminal `[Raspberry]`
```bash
docker-compose -f docker-compose.real.yml down
xhost -local:root
```

## Build Instructions

### Task 1: For Simulated Robot

Go to `docker-compose.yml` file and uncomment build section and comment out the line that pulls image from dockerhub.

#### 1. Build `roscore_gazebo` service
Build, tag, and push the image for `roscore_gazebo` service

Terminal `[PC]`
```bash
cd ~/simulation_ws/src/tortoisebot_ros1_docker
docker-compose build roscore_gazebo
```

Terminal `[PC]`
```bash
docker tag tortoisebot-ros1-gazebo:v1 kkhadka343/kailash-cp22:tortoisebot-ros1-gazebo
docker push kkhadka343/kailash-cp22:tortoisebot-ros1-gazebo
```

#### 2. Build `waypoint_server` service
Build, tag, and push the image for `waypoint_server` service

Terminal `[PC]`
```bash
cd ~/simulation_ws/src/tortoisebot_ros1_docker
docker-compose build roscore_gazebo
```

Terminal `[PC]`
```bash
docker tag tortoisebot-ros1-waypoints:v1 kkhadka343/kailash-cp22:tortoisebot-ros1-waypoints 
docker push kkhadka343/kailash-cp22:tortoisebot-ros1-waypoints
```

#### 3. Build `mapping` service
Build, tag, and push the image for `mapping` service

Terminal `[PC]`
```bash
cd ~/simulation_ws/src/tortoisebot_ros1_docker
docker-compose build roscore_gazebo
```

Terminal `[PC]`
```bash
docker tag tortoisebot-ros1-slam:v1 kkhadka343/kailash-cp22:tortoisebot-ros1-slam
docker push kkhadka343/kailash-cp22:tortoisebot-ros1-slam
```

#### 4. Build `web_server` service
Build, tag, and push the image for `web_server` service

Terminal `[PC]`
```bash
cd ~/simulation_ws/src/tortoisebot_ros1_docker
docker-compose build roscore_gazebo
```

Terminal `[PC]`
```bash
docker tag tortoisebot-ros1-webapp:v1 kkhadka343/kailash-cp22:tortoisebot-ros1-webapp
docker push kkhadka343/kailash-cp22:tortoisebot-ros1-webapp
```

### Task 3: For real robots
The image for real robot to bring up firmware, camera and lidar were built for running in raspberrypi.
Hence, the building process requires QEMU emulator for real robots.

#### 1. Build `roscore_real` service
Go to `docker-compose.real.yml` file and uncomment build section and comment out the line that pulls image from dockerhub.
Build, tag, and push the image for `roscore_real` service

Terminal `[PC]`
```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
cd ~/ros1_ws/src/tortoisebot_ros1_docker
docker-compose -f docker-compose.real.yml build roscore_real
```

Terminal `[PC]`
```bash
docker tag tortoisebot-ros1-real:v1 kkhadka343/kailash-cp22:tortoisebot-ros1-real
docker push kkhadka343/kailash-cp22:tortoisebot-ros1-real
```

#### 2. Build `mapping_real` service
Go to `docker-compose.real.yml` file and uncomment build section and comment out the line that pulls image from dockerhub.
Build, tag, and push the image for `mapping_real` service

Terminal `[PC]`
```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
cd ~/ros1_ws/src/tortoisebot_ros1_docker
docker-compose -f docker-compose.real.yml build mapping_real
```

Terminal `[PC]`
```bash
docker tag tortoisebot-ros1-slam-real:v1 kkhadka343/kailash-cp22:tortoisebot-ros1-slam-real
docker push kkhadka343/kailash-cp22:tortoisebot-ros1-slam-real
```