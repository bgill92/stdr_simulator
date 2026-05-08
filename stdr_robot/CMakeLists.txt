cmake_minimum_required(VERSION 3.16)
project(stdr_robot)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  stdr_msgs
  stdr_simulation
  stdr_parser
  rosidl_default_runtime)

add_executable(robot_spawner src/robot_spawner_main.cpp)

target_compile_features(robot_spawner PUBLIC cxx_std_20)
set_target_properties(robot_spawner PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(
  robot_spawner
  PRIVATE ${PROJECT_NAME} stdr_parser::stdr_parser
          ament_index_cpp::ament_index_cpp ${rclcpp_action_TARGETS})

install(TARGETS robot_spawner RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_robot_node test/test_robot_node.cpp)
  target_link_libraries(test_robot_node ${PROJECT_NAME} GTest::gmock)
endif()

ament_package()
