cmake_minimum_required(VERSION 3.8)
project(project_1)

# Enable strict warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclpy
  std_msgs
)

# Include directory
include_directories(include)

# C++ executables (corrected paths)
add_executable(cpp_minimal_publisher cpp/cpp_minimal_publisher.cpp)
ament_target_dependencies(cpp_minimal_publisher ${DEPENDENCIES})

add_executable(cpp_minimal_subscriber cpp/cpp_minimal_subscriber.cpp)
ament_target_dependencies(cpp_minimal_subscriber ${DEPENDENCIES})

# Install C++ executables
install(
  TARGETS
    cpp_minimal_publisher
    cpp_minimal_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts (corrected)
install(
  PROGRAMS
    py/project_1_publisher.py
    py/project_1_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
