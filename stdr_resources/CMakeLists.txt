cmake_minimum_required(VERSION 3.16)
project(stdr_resources)

find_package(ament_cmake REQUIRED)

install(DIRECTORY resources maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
