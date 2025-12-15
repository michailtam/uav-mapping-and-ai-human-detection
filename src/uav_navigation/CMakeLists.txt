cmake_minimum_required(VERSION 3.8)
project(uav_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## CPP EXECUTABLES
# include_directories(include)

## Create the cpp executables
# add_executable(cpp_node src/cpp_node.cpp)
# ament_target_dependencies(cpp_node rclcpp)

# # Install C++ executables
# install(TARGETS
#   cpp_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python scripts
install(PROGRAMS
  scripts/simple_commander.py
  scripts/simple_commander_2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config maps bt
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
