cmake_minimum_required(VERSION 3.24)
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
  src/panels/toolbar.cpp
  src/plot/helpers.cpp
  src/plot/plot_panel.cpp
  src/plot/plot_sink_view.cpp
  src/plot/registry.cpp
  src/plot/sim_view.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC stdr_simulation::stdr_simulation tl_expected::tl_expected imgui::imgui
         implot::implot OpenGL::GL)

# ---------------------------------------------------------------------------
# stdr_gui_plotters: static library of built-in plotter plugins.
#
# Plotter TUs each contain a file-scope `static bool` initializer produced by
# REGISTER_PLOTTER.  With --gc-sections (ament default), those TUs would be
# silently dead-stripped when linked into a shared library or executable that
# has no direct symbol references.  A STATIC library linked with
# $<LINK_LIBRARY:WHOLE_ARCHIVE,...> forces every TU to be included, so the
# static initializers run and the registry is populated.
#
# Users who write their own plotters should either: (a) add their .cpp to this
# library's GLOB (drop it in src/plotters/), or (b) create their own static
# library and link it with WHOLE_ARCHIVE.
# ---------------------------------------------------------------------------
file(GLOB _plotter_sources CONFIGURE_DEPENDS "src/plotters/*.cpp")

add_library(stdr_gui_plotters STATIC ${_plotter_sources})

target_compile_features(stdr_gui_plotters PUBLIC cxx_std_20)
set_target_properties(stdr_gui_plotters PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(stdr_gui_plotters PUBLIC ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME} stdr_gui_plotters
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

  # PlotSink / PlotView unit tests — only stdr_gui needed (no backend).
  ament_add_gtest(test_plot_sink_view test/test_plot_sink_view.cpp)
  target_link_libraries(test_plot_sink_view ${PROJECT_NAME} GTest::gmock)

  # Registry unit tests — links stdr_gui_plotters with WHOLE_ARCHIVE so
  # REGISTER_PLOTTER static initializers from the test TU and
  # pose_error_plotter.cpp both run and populate the global singleton.
  ament_add_gtest(test_plotter_registry test/test_plotter_registry.cpp)
  target_link_libraries(
    test_plotter_registry ${PROJECT_NAME}
    $<LINK_LIBRARY:WHOLE_ARCHIVE,stdr_gui_plotters> GTest::gmock)

  # Plot helper utilities unit tests.
  ament_add_gtest(test_plot_helpers test/test_plot_helpers.cpp)
  target_link_libraries(test_plot_helpers ${PROJECT_NAME} GTest::gmock)

  # PlotPanel state-machine tests — no ImGui context required. Links stdr_gui
  # only (no WHOLE_ARCHIVE needed since test plotter types are defined inline in
  # the test TU, not in stdr_gui_plotters).
  ament_add_gtest(test_plot_panel test/test_plot_panel.cpp)
  target_link_libraries(test_plot_panel ${PROJECT_NAME} GTest::gmock)

  # SimulatorBackend default effective-rate helper tests.
  ament_add_gtest(test_simulator_backend test/test_simulator_backend.cpp)
  target_link_libraries(test_simulator_backend ${PROJECT_NAME} GTest::gmock)
endif()

ament_package()
