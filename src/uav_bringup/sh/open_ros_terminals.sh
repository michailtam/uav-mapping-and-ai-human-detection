#!/bin/bash

# Terminal 1 (named & minimized): Start QGroundControl
# gnome-terminal --title="QGC" --window --geometry=80x24+0+0 -- bash -c "../QGroundControl-x86_64.AppImage exec bash" &
# wmctrl -r "QGC" -b add,hidden

# Terminal 2 (named & minimized): Start PX4 SITL
gnome-terminal --title="PX4 SITL" --window --geometry=80x24+0+0 -- bash -c "source install/setup.bash && ros2 launch uav_bringup start_sitl.launch.py; exec bash" &
wmctrl -r "PX4 SITL" -b add,hidden

# Terminal 3 (named & minimized): Start communication between PX4 with ROS 2
gnome-terminal --title="Micro-XRCE-DDS-Agent" --window --geometry=80x24+0+0 -- bash -c "source install/setup.bash && ros2 launch uav_offboard_ctrl xrce_dds_agent.launch.py; exec bash" &
wmctrl -r "Micro-XRCE-DDS-Agent" -b add,hidden

# Terminal 4 (named & minimized): Start communication between ROS 2 and Gazebo
gnome-terminal --title="ROS-GZ-Bridge" --window --geometry=80x24+0+0 -- bash -c "source install/setup.bash && ros2 launch uav_simulation ros_gz_bridge.launch.py; exec bash" &
wmctrl -r "ROS GZ Bridge" -b add,hidden

# # Terminal 5 (named & minimized): Start Nav 2 and SLAM
gnome-terminal --title="SLAM-Navigation" --window --geometry=80x24+0+0 -- bash -c "source install/setup.bash && ros2 launch uav_navigation slam_and_nav.launch.py; exec bash" &
wmctrl -r "SLAM-Navigation" -b add,hidden

