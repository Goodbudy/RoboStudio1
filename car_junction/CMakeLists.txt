cmake_minimum_required(VERSION 3.5)
project(car_junction)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()

