cmake_minimum_required(VERSION 3.8)
project(lab5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(vector REQUIRED)

add_executable(lab5 src/lab5.cpp)
ament_target_dependencies(lab5 rclcpp sensor_msgs std_msgs geometry_msgs OpenCV)

install(TARGETS
lab5
DESTINATION lib/${PROJECT_NAME})
ament_package()
