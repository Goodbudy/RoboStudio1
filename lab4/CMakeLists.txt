cmake_minimum_required(VERSION 3.8)
project(lab4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(lab4 src/Dead_reckoning.cpp)
ament_target_dependencies(lab4 rclcpp nav_msgs std_msgs geometry_msgs)


install(TARGETS
lab4
DESTINATION lib/${PROJECT_NAME})


ament_package()