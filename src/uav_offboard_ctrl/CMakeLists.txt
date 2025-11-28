cmake_minimum_required(VERSION 3.8)
project(uav_offboard_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add include folder so .h files are found
include_directories(include)

add_executable(px4_ctrl src/px4_ctrl.cpp)
ament_target_dependencies(px4_ctrl px4_msgs rclcpp)

add_executable(px4_prop_js_publisher src/px4_prop_js_publisher.cpp)
ament_target_dependencies(px4_prop_js_publisher px4_msgs rclcpp sensor_msgs)

# Install the header files
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install the packages and executables
install(TARGETS
  px4_prop_js_publisher
  px4_ctrl
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
