cmake_minimum_required(VERSION 3.16)
project(stdr_launchers)

find_package(catkin REQUIRED)

catkin_package()

# Install launch files
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
