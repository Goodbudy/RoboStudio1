cmake_minimum_required(VERSION 3.5)
project(lab2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(lab2 src/lab2.cpp)

# Link libraries
ament_target_dependencies(lab2 rclcpp sensor_msgs)

# Install executable
install(TARGETS
  lab2
  DESTINATION lib/${PROJECT_NAME})

# ament package declaration
ament_package()
