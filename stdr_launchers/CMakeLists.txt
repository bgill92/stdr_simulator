cmake_minimum_required(VERSION 3.16)
project(stdr_launchers)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
