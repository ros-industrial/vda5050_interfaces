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
  "msg/AGVPosition.msg"
  "msg/ActionState.msg"
  "msg/BatteryState.msg"
  "msg/BoundingBoxReference.msg"
  "msg/Connection.msg"
  "msg/ControlPoint.msg"
  "msg/EdgeState.msg"
  "msg/Error.msg"
  "msg/ErrorReference.msg"
  "msg/Header.msg"
  "msg/InfoReference.msg"
  "msg/Information.msg"
  "msg/Load.msg"
  "msg/LoadDimensions.msg"
  "msg/NodePosition.msg"
  "msg/NodeState.msg"
  "msg/SafetyState.msg"
  "msg/State.msg"
  "msg/Trajectory.msg"
  "msg/Velocity.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ADD_LINTER_TESTS
)

install(
  DIRECTORY include/vda5050_msgs/json_utils
  DESTINATION include/vda5050_msgs/vda5050_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  nlohmann_json
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()

  find_package(ament_cmake_cpplint REQUIRED)
  set(cpplint_filters
    "-whitespace/newline"
  )
  set(cpplint_root_paths
    "${CMAKE_CURRENT_SOURCE_DIR}/include/"
  )

  ament_cpplint(
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

  # Get C++ typesupport target to be able to link generated message types
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

  ament_add_gtest(${PROJECT_NAME}_test test/test_json_utils.cpp)
  target_include_directories(${PROJECT_NAME}_test
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
  target_link_libraries(${PROJECT_NAME}_test
    nlohmann_json::nlohmann_json
    ${cpp_typesupport_target}
  )
endif()

ament_package()

