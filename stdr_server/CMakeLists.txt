cmake_minimum_required(VERSION 3.16)
project(stdr_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(stdr_msgs REQUIRED)
find_package(stdr_simulation REQUIRED)
find_package(stdr_parser REQUIRED)
find_package(tl_expected REQUIRED)
find_package(PNG REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_library(${PROJECT_NAME} src/map_loader.cpp src/stdr_server_node.cpp)

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
         ${nav_msgs_TARGETS}
         ${geometry_msgs_TARGETS}
         tf2_ros::tf2_ros
         ${visualization_msgs_TARGETS}
         ${stdr_msgs_TARGETS__rosidl_typesupport_cpp}
         stdr_simulation::stdr_simulation
         stdr_parser::stdr_parser
         tl_expected::tl_expected)

target_link_libraries(${PROJECT_NAME} PRIVATE PNG::PNG)

# Standalone server executable
add_executable(stdr_server_node src/main.cpp)
target_link_libraries(stdr_server_node PRIVATE ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS stdr_server_node DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_msgs
  geometry_msgs
  tf2_ros
  visualization_msgs
  stdr_msgs
  stdr_simulation
  stdr_parser
  tl_expected
  rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_map_loader test/test_map_loader.cpp)
  target_link_libraries(test_map_loader ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_server_node test/test_server_node.cpp)
  target_link_libraries(test_server_node ${PROJECT_NAME} GTest::gmock)
endif()

ament_package()
