cmake_minimum_required(VERSION 3.16)
project(stdr_simulation)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/config_loader.cpp
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
  src/sensors/sound_simulator.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp::yaml-cpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_config_loader
    test/test_config_loader.cpp
  )
  target_compile_definitions(test_config_loader PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )
  target_link_libraries(test_config_loader ${PROJECT_NAME} GTest::gmock)
endif()

ament_package()
