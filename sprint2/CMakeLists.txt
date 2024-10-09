cmake_minimum_required(VERSION 3.8)
project(sprint2)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
 
# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
 
# Define executables
add_executable(sprint2 src/sprint2.cpp)
 
# Link dependencies to each target
ament_target_dependencies(sprint2 rclcpp std_srvs tf2_geometry_msgs tf2 nav_msgs geometry_msgs std_msgs sensor_msgs cv_bridge image_transport OpenCV)
 
# Link libraries for OpenCV
target_link_libraries(sprint2 ${OpenCV_LIBRARIES})
 
# Install executables
install(TARGETS
  sprint2
  DESTINATION lib/${PROJECT_NAME}
)
 
ament_package()