cmake_minimum_required(VERSION 3.16)
project(stdr_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(stdr_msgs REQUIRED)
find_package(stdr_simulation REQUIRED)
find_package(stdr_parser REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_library(${PROJECT_NAME} src/stdr_robot_node.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC rclcpp::rclcpp
         ${rclcpp_action_TARGETS}
         ${rclcpp_components_TARGETS}
         tf2::tf2
         tf2_ros::tf2_ros
         ${nav_msgs_TARGETS}
         ${sensor_msgs_TARGETS}
         ${geometry_msgs_TARGETS}
         ${stdr_msgs_TARGETS__rosidl_typesupport_cpp}
         stdr_simulation::stdr_simulation
         stdr_parser::stdr_parser)

rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN "stdr_robot::StdrRobotNode" EXECUTABLE stdr_robot_node)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_robot_node test/test_robot_node.cpp)
  target_link_libraries(test_robot_node ${PROJECT_NAME} GTest::gmock)
endif()

ament_package()
