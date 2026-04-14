cmake_minimum_required(VERSION 3.16)
project(stdr_gui)

find_package(ament_cmake REQUIRED)
find_package(stdr_simulation REQUIRED)
find_package(tl_expected REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

add_library(
  ${PROJECT_NAME}
  src/grid_utils.cpp
  src/gui_app.cpp
  src/map_transform.cpp
  src/pose_utils.cpp
  src/teleop_controller.cpp
  src/panels/file_dialog.cpp
  src/panels/map_panel.cpp
  src/panels/robot_info_panel.cpp
  src/panels/sensor_window.cpp
  src/panels/toolbar.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC stdr_simulation::stdr_simulation tl_expected::tl_expected imgui::imgui
         implot::implot OpenGL::GL)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(stdr_simulation tl_expected imgui implot OpenGL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_map_transform test/test_map_transform.cpp)
  target_link_libraries(test_map_transform ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_format_elapsed_time test/test_format_elapsed_time.cpp)
  target_link_libraries(test_format_elapsed_time ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_grid_utils test/test_grid_utils.cpp)
  target_link_libraries(test_grid_utils ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_pose_utils test/test_pose_utils.cpp)
  target_link_libraries(test_pose_utils ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_file_dialog test/test_file_dialog.cpp)
  target_link_libraries(test_file_dialog ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_teleop_controller test/test_teleop_controller.cpp)
  target_link_libraries(test_teleop_controller ${PROJECT_NAME} GTest::gmock)
endif()

ament_package()
