# AI-Driven UAV Simulation for Semantic Mapping and Human Detection with ROS 2, Gazebo, and Computer Vision

[![Status](https://img.shields.io/badge/Status-Stable-blue.svg)](#)
[![Degree](https://img.shields.io/badge/Degree-M.Sc._Thesis-blue.svg)](#)
[![Domain](https://img.shields.io/badge/Domain-Computer_Science_%26_Software_Eng-informational.svg)](#)

## Description
**Note💡:** Will be coming soon.

## System Setup
Before running the application, ensure your system meets the OS requirements and has the necessary packages installed.

> **Note:** This project requires **[Ubuntu 24.04 LTS](https://releases.ubuntu.com/noble/)** with **[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)** installed.

### **1️⃣** System Dependencies

Update your package lists and install the required automation tools (`wmctrl` and `xdotool`):
```bash
sudo apt update && sudo apt upgrade -y && sudo apt install -y \
  python3-pip \
  wmctrl \
  xdotool
```

### **2️⃣** Install additional ROS 2 packages
```bash
sudo apt update && sudo apt install -y \
  ros-$ROS_DISTRO-joy \
  ros-$ROS_DISTRO-joy-teleop \
  ros-$ROS_DISTRO-message-tf-frame-transformer \
  ros-$ROS_DISTRO-odom-to-tf-ros2 \
  ros-$ROS_DISTRO-rqt \
  ros-$ROS_DISTRO-tf2-tools \
  ros-$ROS_DISTRO-urdf-tutorial \
  ros-$ROS_DISTRO-xacro
```

### **3️⃣** Install the keyboard teleop control
* [pynput 1.8.2](https://pypi.org/project/pynput/)

### **4️⃣** Install Gazebo Harmonic 
* [Binary Installation on Ubuntu](https://gazebosim.org/docs/harmonic/install_ubuntu/)

### **5️⃣** Export models and install additional gazebo packages
```bash
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$HOME/.simulation-gazebo/models
sudo apt install ros-$ROS_DISTRO-ros-gz-bridge \
  ros-$ROS_DISTRO-ros-gz-sim \
  ros-$ROS_DISTRO-ros-gz-image \
  ros-$ROS_DISTRO-ros-gz-sim-demos
```

### **6️⃣** Install QGroundControl 
[Download and Install](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html)

### **7️⃣** Install Computer Vision packages
* [YOLO8](https://docs.ultralytics.com/models/yolov8#overview)
```bash
 pip3 install ultralytics

# Note: If the terminal outputs externally-managed-environment, refer to the following links to fix the issue:
# https://www.youtube.com/watch?v=g2TDfWDgwkE
# https://stackoverflow.com/questions/38869231/python-cant-find-the-file-pip-conf
# https://stackoverflow.com/questions/75608323/how-do-i-solve-error-externally-
# managed-environment-every-time-i-use-pip-3

sudo apt-get install ros-$ROS_DISTRO-vision-opencv
sudo apt update && sudo apt install -y \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-ros-gz-image
```

### **8️⃣** Install RTAB Map
```bash
sudo apt update && sudo apt install -y \
  libpcl-dev \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-grid-map-ros \
  ros-$ROS_DISTRO-librealsense2 \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-realsense2-camera \
  ros-$ROS_DISTRO-realsense2-description \
  ros-$ROS_DISTRO-ros-gz-image \
  ros-$ROS_DISTRO-aruco-opencv \
  ros-$ROS_DISTRO-rqt-image-view \
  ros-$ROS_DISTRO-rtabmap-ros \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-rtabmap-rviz-plugins \
  ros-$ROS_DISTRO-rtabmap-viz \
  ros-$ROS_DISTRO-stereo-image-proc
```

## Project Setup

### **1️⃣** Clone the project from the repository
```bash
git clone --recurse-submodules https://github.com/michailtam/uav-mapping-and-ai-human-detection.git
```

### **2️⃣** Setup Autopilot for PX4
```bash
git clone https://github.com/PX4/PX4-Autopilot.git ./external/PX4-Autopilot --recursive
bash ./external/PX4-Autopilot/Tools/setup/ubuntu.sh

# Test the simulation using the x500 drone.
cd ./external/PX4-Autopilot
git submodule update --init --recursive

# Clear previous build artifacts.
rm -rf build/

# Build again with local install prefix.
make px4_sitl gz_x500 CMAKE_INSTALL_PREFIX=$(pwd)/build/px4_sitl_default/install
```

### **3️⃣** Setup uORB to ROS 2 Message Translation via Micro-XRCE-DDS
```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ./external/Micro-XRCE-DDS-Agent
cd ./external/Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
cd ../../
```

### **4️⃣** Setup PX4 messages
```bash
git clone https://github.com/PX4/px4_msgs.git ./external/px4_msgs
```

### **5️⃣** Setup PX4-ROS 2 bridge
```bash
git clone https://github.com/PX4/px4_ros_com.git ./external/px4_ros_com
```

### **6️⃣** Setup the simulator to use the custom drone x650 instead of the x500 
```bash
# 1. Copy the x650 content to the PX4-Autopilot models folder.
cp -r ./external/config/x650 external/PX4-Autopilot/Tools/simulation/gz/models/

# 2. Copy the custom airframe startup file to PX4 ROMFS.
cp ./external/config/4229_gz_x650 external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

# 3. Grant executable permissions and verify.
chmod +x external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4229_gz_x650
ls -l ./external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4229_gz_x650

# 4. Copy directly into the active build rootfs (for immediate testing without full rebuild).
cp ./external/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4229_gz_x650 ./external/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/
chmod +x ./external/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/4229_gz_x650

# 5. Inform gazebo where to find the model and do a test run if everything works.
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/external/PX4-Autopilot/Tools/simulation/gz/models

# 6. Test the settings
# Note: Gazebo should spawn the x650 drone in the environment baylands. 
PX4_SYS_AUTOSTART=4229 PX4_GZ_MODEL=x650 ./external/PX4-Autopilot/build/px4_sitl_default/bin/px4

# Note: To kill all processes issue: $ pkill -9 -f px4; pkill -9 -f gz
```

## Docker
**Note💡:** Will be coming soon.

## Run the project
Move to the root folder of the project and execute the following **step-by-step**.
**Note💡:** Please be patient while the application loads and launches the simulator and all required packages.

### Terminal 1: Execute the shell script
Launch PX4 SITL and Gazebo in the background, bridge ROS 2 and Gazebo communication, and initialize RTAB-Map 3D mapping with real-time AI object detection.
```bash
# Note: Previously, clear any previous build, log and install folders with rm -rf install/ log/ build/
colcon build
source install/setup.bash
./src/uav_bringup/sh/open_ros_terminals.sh
```

### Terminal 2:
Launch the px4_ctrl executable from the uav_offboard_ctrl package using simulation time to handle flight commands and setpoint streaming to PX4.
```bash
source install/setup.bash
ros2 launch uav_offboard_ctrl offboard_ctrl.launch.py
```

### Terminal 3:
Launch the keyboard_teleop node from the uav_navigation package to allow manual control and steering of the drone using keyboard commands.
```bash
source install/setup.bash
ros2 run uav_navigation keyboard_teleop.py
```

### Kill all the windows
```bash
pkill -f ros && pkill -f gz && pkill -f gazebo && pkill –f px4pkill -f ros && pkill -f gz \
&& pkill -f gazebo && pkill –f px4
```

## Screenshots
**Note💡:** Will be coming soon.
