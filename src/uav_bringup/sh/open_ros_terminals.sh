#!/bin/bash

# Terminal 1 (named & minimized): Start PX4 SITL
gnome-terminal --title="PX4 SITL" --window --geometry=80x24+0+0 -- bash -c "source install/setup.bash && ros2 launch uav_bringup start_sitl.launch.py; exec bash" &
wmctrl -r "ROS SITL" -b add,hidden

# Terminal 2 (named & minimized): Start communication between PX4 with ROS 2
gnome-terminal --title="Micro-XRCE-DDS-Agent" --window --geometry=80x24+0+0 -- bash -c "source install/setup.bash && ros2 launch uav_offboard_ctrl xrce_dds_agent.launch.py; exec bash" &
wmctrl -r "ROS Offboard Ctrl" -b add,hidden

# Terminal 3 (named & minimized): Start communication between ROS 2 and Gazebo
gnome-terminal --title="ROS-GZ-Bridge" --window --geometry=80x24+0+0 -- bash -c "source install/setup.bash && ros2 launch uav_simulation ros_gz_bridge.launch.py; exec bash" &
wmctrl -r "ROS GZ Bridge" -b add,hidden

# Terminal 4 (named & minimized): Start Nav 2 and SLAM
gnome-terminal --title="Navigation" --window --geometry=80x24+0+0 -- bash -c "source install/setup.bash && ros2 launch uav_navigation slam.launch.py; exec bash" &
wmctrl -r "Nav2 SLAM" -b add,hidden
