cmake_minimum_required(VERSION 3.16)
project(stdr_resources)

find_package(catkin REQUIRED)

catkin_package()

# Install files
install(DIRECTORY maps resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


