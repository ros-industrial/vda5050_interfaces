cmake_minimum_required(VERSION 3.8)
project(vda5050_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)

# Generate VDA5050 ROS 2 messages
set(msg_files
  "msg/Connection.msg"
  "msg/Header.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ADD_LINTER_TESTS
)

# Get C++ typesupport target to be able to link generated message types
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Create serialization/deserialization library
set(json_util_files
  "src/vda5050_msgs/json_utils/connection.cpp"
  "src/vda5050_msgs/json_utils/header.cpp"
)
add_library(json_utils STATIC ${json_util_files})
target_include_directories(json_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(json_utils
  PUBLIC
    nlohmann_json::nlohmann_json
    ${cpp_typesupport_target}
)

install(
  DIRECTORY include/vda5050_msgs/json_utils
  DESTINATION include/vda5050_msgs/vda5050_msgs
)

install(
  TARGETS json_utils
  EXPORT json_utils
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Export CMake target for the library under the vda5050_msgs namespace
install(
  EXPORT json_utils
  NAMESPACE vda5050_msgs::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(json_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rosidl_default_generators
  rosidl_default_runtime
  nlohmann_json
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()

  find_package(ament_cmake_cpplint REQUIRED)
  set(cpplint_exclude_paths
    "${CMAKE_CURRENT_SOURCE_DIR}/test/"
  )
  set(cpplint_filters
    "-whitespace/newline"
    "-build/namespaces"
  )
  set(cpplint_root_paths
    "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
  )

  ament_cpplint(
    EXCLUDE "${cpplint_exclude_paths}"
    FILTERS "${cpplint_filters}"
    ROOT "${cpplint_root_paths}"
  )

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format"
  )

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosidl_generator_cpp REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test test/test_json_utils.cpp)
  target_include_directories(${PROJECT_NAME}_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
  target_link_libraries(${PROJECT_NAME}_test json_utils)
  ament_target_dependencies(${PROJECT_NAME}_test rosidl_generator_cpp)
endif()

ament_package()

