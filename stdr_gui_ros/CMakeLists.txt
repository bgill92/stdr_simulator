cmake_minimum_required(VERSION 3.16)
project(stdr_gui_ros)

find_package(ament_cmake REQUIRED)
find_package(stdr_gui REQUIRED)
find_package(stdr_simulation REQUIRED)
find_package(stdr_msgs REQUIRED)
find_package(stdr_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tl_expected REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} src/ros2_backend.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME} PUBLIC stdr_gui::stdr_gui stdr_simulation::stdr_simulation
                         tl_expected::tl_expected Threads::Threads)

ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  stdr_msgs
  stdr_parser
  tf2
  tf2_ros)

add_executable(stdr_gui_ros_node src/main.cpp)

target_compile_features(stdr_gui_ros_node PRIVATE cxx_std_20)
set_target_properties(stdr_gui_ros_node PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(stdr_gui_ros_node PRIVATE ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS stdr_gui_ros_node DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  stdr_gui
  stdr_simulation
  stdr_msgs
  stdr_parser
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tl_expected)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_ros2_backend test/test_ros2_backend.cpp)
  target_link_libraries(test_ros2_backend ${PROJECT_NAME} GTest::gmock_main)
  target_compile_definitions(
    test_ros2_backend
    PRIVATE STDR_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../stdr_resources")
endif()

ament_package()
