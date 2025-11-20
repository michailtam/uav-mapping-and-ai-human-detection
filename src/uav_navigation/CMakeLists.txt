cmake_minimum_required(VERSION 3.8)
project(uav_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build system
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# C++ dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(tf2_eigen REQUIRED)

# For Python nodes and to install them as a package, uncomment:
# ament_python_install_package(${PROJECT_NAME})

# C++ executable: ROS odom -> PX4 VehicleOdometry
add_executable(ros_odometry_to_vehicle_odometry
  src/ros_odometry_to_vehicle_odometry.cpp
)

# Link against the px4_ros_com frame_transforms library
target_link_libraries(ros_odometry_to_vehicle_odometry
  px4_ros_com::frame_transforms
)

ament_target_dependencies(ros_odometry_to_vehicle_odometry
  rclcpp
  px4_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
)

install(TARGETS
  ros_odometry_to_vehicle_odometry
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
