cmake_minimum_required(VERSION 3.16)
project(stdr_standalone)

find_package(ament_cmake REQUIRED)
find_package(stdr_simulation REQUIRED)
find_package(stdr_gui REQUIRED)
find_package(tl_expected REQUIRED)
find_package(PNG REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} src/standalone_backend.cpp src/map_loader.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC stdr_simulation::stdr_simulation stdr_gui::stdr_gui
         tl_expected::tl_expected Threads::Threads
  PRIVATE PNG::PNG)

add_executable(stdr_standalone_exe src/main.cpp)

target_compile_features(stdr_standalone_exe PRIVATE cxx_std_20)
set_target_properties(stdr_standalone_exe PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(stdr_standalone_exe PRIVATE ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS stdr_standalone_exe DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(stdr_simulation stdr_gui tl_expected)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_standalone_backend test/test_standalone_backend.cpp)
  target_link_libraries(test_standalone_backend ${PROJECT_NAME} GTest::gmock)
  target_compile_definitions(
    test_standalone_backend
    PRIVATE STDR_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../stdr_resources"
            STDR_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest(test_map_loader test/test_map_loader.cpp)
  target_link_libraries(test_map_loader ${PROJECT_NAME} GTest::gmock)
  target_compile_definitions(
    test_map_loader
    PRIVATE STDR_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../stdr_resources")
endif()

ament_package()
