cmake_minimum_required(VERSION 3.16)
project(stdr_samples)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(obstacle_avoidance src/obstacle_avoidance/main.cpp
                                  src/obstacle_avoidance/obstacle_avoidance.cpp)

target_compile_features(obstacle_avoidance PRIVATE cxx_std_20)
set_target_properties(obstacle_avoidance PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(obstacle_avoidance
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
  obstacle_avoidance PRIVATE rclcpp::rclcpp ${geometry_msgs_TARGETS}
                             ${sensor_msgs_TARGETS})

install(TARGETS obstacle_avoidance RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
