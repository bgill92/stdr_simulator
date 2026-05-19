cmake_minimum_required(VERSION 3.16)
project(stdr_simulation)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tl_expected REQUIRED)

add_library(
  ${PROJECT_NAME}
  src/simulation_engine.cpp
  src/geometry_utils.cpp
  src/config_loader.cpp
  src/rate_scheduler.cpp
  src/world/world_model.cpp
  src/collision/collision_checker.cpp
  src/motion/noise_model.cpp
  src/motion/ideal_motion_model.cpp
  src/motion/omni_motion_model.cpp
  src/sensors/laser_simulator.cpp
  src/sensors/sonar_simulator.cpp
  src/sensors/rfid_simulator.cpp
  src/sensors/co2_simulator.cpp
  src/sensors/thermal_simulator.cpp
  src/sensors/sound_simulator.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp::yaml-cpp
                                             tl_expected::tl_expected)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp tl_expected)

# Benchmarks are off by default so they do not slow down CI.  Enable with
# -DBUILD_BENCHMARKS=ON when profiling locally.
option(BUILD_BENCHMARKS "Build Google Benchmark targets" OFF)
if(BUILD_BENCHMARKS)
  find_package(ament_cmake_google_benchmark QUIET)
  if(ament_cmake_google_benchmark_FOUND)
    ament_add_google_benchmark(bench_plot_data benchmark/bench_plot_data.cpp)
    target_compile_features(bench_plot_data PRIVATE cxx_std_20)
    set_target_properties(bench_plot_data PROPERTIES CXX_EXTENSIONS OFF)
    target_include_directories(
      bench_plot_data
      PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  else()
    message(
      STATUS "BUILD_BENCHMARKS=ON but ament_cmake_google_benchmark not found — "
             "skipping bench_plot_data target")
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_config_loader test/test_config_loader.cpp)
  target_compile_definitions(
    test_config_loader
    PRIVATE FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  target_link_libraries(test_config_loader ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_geometry_utils test/test_geometry_utils.cpp)
  target_link_libraries(test_geometry_utils ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_collision_checker test/test_collision_checker.cpp)
  target_link_libraries(test_collision_checker ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_motion_models test/test_motion_models.cpp)
  target_link_libraries(test_motion_models ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_sensors test/test_sensors.cpp)
  target_link_libraries(test_sensors ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_world_model test/test_world_model.cpp)
  target_link_libraries(test_world_model ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_simulation_engine test/test_simulation_engine.cpp)
  target_link_libraries(test_simulation_engine ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_rate_scheduler test/test_rate_scheduler.cpp)
  target_link_libraries(test_rate_scheduler ${PROJECT_NAME} GTest::gmock)

  # plot_data primitives — header-only, no link to ${PROJECT_NAME} needed.
  ament_add_gtest(test_spsc_ring test/test_spsc_ring.cpp)
  target_compile_features(test_spsc_ring PRIVATE cxx_std_20)
  set_target_properties(test_spsc_ring PROPERTIES CXX_EXTENSIONS OFF)
  target_link_libraries(test_spsc_ring GTest::gmock)
  target_include_directories(
    test_spsc_ring
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  ament_add_gtest(test_latest_snapshot test/test_latest_snapshot.cpp)
  target_compile_features(test_latest_snapshot PRIVATE cxx_std_20)
  set_target_properties(test_latest_snapshot PROPERTIES CXX_EXTENSIONS OFF)
  target_link_libraries(test_latest_snapshot GTest::gmock)
  target_include_directories(
    test_latest_snapshot
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  # command_queue is header-only (mutex-guarded, no compiled sources).
  ament_add_gtest(test_command_queue test/test_command_queue.cpp)
  target_compile_features(test_command_queue PRIVATE cxx_std_20)
  set_target_properties(test_command_queue PROPERTIES CXX_EXTENSIONS OFF)
  target_link_libraries(test_command_queue GTest::gmock)
  target_include_directories(
    test_command_queue
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
endif()

ament_package()
