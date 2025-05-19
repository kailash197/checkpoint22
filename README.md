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

# Task4: ROS2 SLAM in real robot [Tortoisebot]

## Run Instructions

### 1. Connect to Tortoisebot

Terminal[PC]
```bash
ssh -X tortoisebot@<ip of raspberrypi>
```

### 2. Install docker, docker-compose and xhost in Tortoisebot
Please follow installation steps: [[Install docker & xhost](https://github.com/kailash197/checkpoint22/tree/ros2-galactic?tab=readme-ov-file#installation-steps)].

### 3. Clone the repo
Terminal [RASPBERRYPI]:
```bash
mkdir -p ~/ros2_ws/src # if doesn't exist yet
cd ~/ros2_ws/src
git clone -b ros2-galactic https://github.com/kailash197/checkpoint22.git
mv checkpoint22 tortoisebot_ros2_docker
cd ~/ros2_ws/src/tortoisebot_ros2_docker
```

### 4. Pull images from dockerhub
Command to manually pull these images from dockerhub:
Terminal [RASPBERRYPI]:
```bash
docker pull kkhadka343/kailash-cp22:tortoisebot-ros2-real
docker pull kkhadka343/kailash-cp22:tortoisebot-ros2-slam-real
```

### 5. Start all services
Start all the necessary services.

Terminal [RASPBERRYPI]:
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose -f docker-compose.real.yml up -d
```
This will start both services and both containers will be available.
First, all the sensors and actuators on the robot will be launced automatically through the service `ros2_real`, container `ros2_real_container`.
Second, SLAM services, includes cartographer node, will be loaded into `mapping2_real` service, container `mapping2_real_container`.

#### Verify the required containers

```bash
docker ps
```
Sample output:
```bash
CONTAINER ID   IMAGE                                                COMMAND                  CREATED              STATUS              PORTS     NAMES
187c62b7404b   kkhadka343/kailash-cp22:tortoisebot-ros2-slam-real   "/entrypoint.sh tail…"   About a minute ago   Up About a minute             slam2_real_container
07617aac5278   kkhadka343/kailash-cp22:tortoisebot-ros2-real        "/entrypoint.sh /bin…"   About a minute ago   Up About a minute             ros2_real_container
```

### 6. Start Mapping program
Run the following command in the tortoisebot to start Mapping program in `mapping2_real_container` container of `mapping2_real` service.

Terminal [RASPBERRYPI]: Run in background
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose -f docker-compose.real.yml \
    exec slam2_real bash -c \
    "source /opt/ros/galactic/setup.bash \
    && source /home/ttbot/ros2_ws/install/setup.bash \
    && ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True" &

```

### 7. RVIZ2 to visualize mapping process
Visualize the robot mapping the environment through RVIZ2. Run the following in PC.

Terminal [PC]
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
source /opt/ros/${ROS_DISTRO}/setup.bash \
&& source ~/ros2_ws/install/setup.bash \
&& ros2 launch tortoisebot_description rviz.launch.py
```

### 8. Move tortoisebot around
Launch `teleop_twist_keyboard` command in `ros2_real_container` to move the robot around and continue mapping process.

Terminal [RASPBERRYPI]
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose -f docker-compose.real.yml \
    exec ros2_real bash -c \
    "source /opt/ros/galactic/setup.bash \
    && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

### 9. Stop services

Terminal [RASPBERRYPI]
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose -f docker-compose.real.yml down
```

# Build Instructions

## Task 2

### [<font color=red>Optional</font>] Build all images
The docker-compose file is designed to pull docker images from docker hub, so it is not necessary to build the images.
If necessary, uncomment build section in `docker-compose.yml` and comment `image` and proceed with build procedure.

Terminal [PC]: Clone the repository
```bash
mkdir -p ~/ros2_ws/src # if doesn't exist yet
cd ~/ros2_ws/src
rm -rf tortoisebot_ros2_docker

git clone -b ros2-galactic https://github.com/kailash197/checkpoint22.git
mv checkpoint22 tortoisebot_ros2_docker

# Clone custom tortoisebot repo
rm -rf tortoisebot
git clone -b ros2-galactic https://github.com/kailash197/cp22-tortoisebot.git
mv cp22-tortoisebot tortoisebot
```

Terminal [PC]: Build `gazebo` service and push to dockerhub
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose build gazebo

docker tag tortoisebot-ros2-gazebo:v1 kkhadka343/kailash-cp22:tortoisebot-ros2-gazebo
docker push kkhadka343/kailash-cp22:tortoisebot-ros2-gazebo
```

Terminal [PC]: Build `slam` service and push to dockerhub
```bash
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose build slam

docker tag tortoisebot-ros2-slam:v1 kkhadka343/kailash-cp22:tortoisebot-ros2-slam
docker push kkhadka343/kailash-cp22:tortoisebot-ros2-slam
```


## Task 4

### [<font color=red>Optional</font>] Build all images
The docker-compose file is designed to pull docker images from docker hub, so it is not necessary to build the images.
  
If necessary, uncomment build section in `docker-compose.real.yml` and comment `image` and proceed with build procedure.
To build ARM64 architechture in PC, reset QEMU static interpreters, Emulator for ARM64-RaspberryPi and then build using docker-compose command.


Terminal [PC]: Clone the repository if not cloned already
```bash
mkdir -p ~/ros2_ws/src # if doesn't exist yet
cd ~/ros2_ws/src
rm -rf tortoisebot_ros2_docker

git clone -b ros2-galactic https://github.com/kailash197/checkpoint22.git
mv checkpoint22 tortoisebot_ros2_docker

# Clone custom tortoisebot repo
rm -rf tortoisebot
git clone -b ros2-galactic https://github.com/kailash197/cp22-tortoisebot.git
mv cp22-tortoisebot tortoisebot
```

Terminal [PC]: Build `ros2_real` service and push to dockerhub
```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose -f docker-compose.real.yml build ros2_real

docker tag tortoisebot-ros2-real:v1 kkhadka343/kailash-cp22:tortoisebot-ros2-real
docker push kkhadka343/kailash-cp22:tortoisebot-ros2-real
```

Terminal [PC]: Build `slam2_real` service and push to dockerhub
```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
cd ~/ros2_ws/src/tortoisebot_ros2_docker
docker-compose -f docker-compose.real.yml build slam2_real

docker tag tortoisebot-ros2-slam-real:v1 kkhadka343/kailash-cp22:tortoisebot-ros2-slam-real
docker push kkhadka343/kailash-cp22:tortoisebot-ros2-slam-real
```
