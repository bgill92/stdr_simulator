cmake_minimum_required(VERSION 3.16)
project(stdr_parser)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tl_expected REQUIRED)
find_package(stdr_simulation REQUIRED)
find_package(stdr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_library(${PROJECT_NAME}
  src/msg_conversions.cpp
  src/yaml_validator.cpp
  src/yaml_writer.cpp
  src/resource_resolver.cpp
  src/parser.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  yaml-cpp::yaml-cpp
  tl_expected::tl_expected
  stdr_simulation::stdr_simulation
  ${stdr_msgs_TARGETS__rosidl_typesupport_cpp}
  ${geometry_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp tl_expected stdr_simulation stdr_msgs geometry_msgs ament_index_cpp rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)

  ament_add_gtest(test_msg_conversions test/test_msg_conversions.cpp)
  target_link_libraries(test_msg_conversions ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_yaml_validator test/test_yaml_validator.cpp)
  target_compile_definitions(test_yaml_validator PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )
  target_link_libraries(test_yaml_validator ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_yaml_writer test/test_yaml_writer.cpp)
  target_compile_definitions(test_yaml_writer PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )
  target_link_libraries(test_yaml_writer ${PROJECT_NAME} GTest::gmock)

  ament_add_gtest(test_parser test/test_parser.cpp)
  target_compile_definitions(test_parser PRIVATE
    FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )
  target_link_libraries(test_parser ${PROJECT_NAME} GTest::gmock)
endif()

ament_package()
