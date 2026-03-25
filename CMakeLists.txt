cmake_minimum_required(VERSION 3.8)
project(vda5050_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate VDA5050 ROS 2 messages
set(msg_files
  "msg/Action.msg"
  "msg/ActionParameter.msg"
  "msg/ActionParameterFactsheet.msg"
  "msg/ActionState.msg"
  "msg/AGVAction.msg"
  "msg/AGVGeometry.msg"
  "msg/AGVPosition.msg"
  "msg/BatteryState.msg"
  "msg/BlockingType.msg"
  "msg/BoundingBoxReference.msg"
  "msg/Connection.msg"
  "msg/ControlPoint.msg"
  "msg/Edge.msg"
  "msg/EdgeState.msg"
  "msg/Envelope2d.msg"
  "msg/Envelope3d.msg"
  "msg/Error.msg"
  "msg/ErrorReference.msg"
  "msg/Factsheet.msg"
  "msg/Header.msg"
  "msg/InstantActions.msg"
  "msg/Info.msg"
  "msg/InfoReference.msg"
  "msg/Load.msg"
  "msg/LoadDimensions.msg"
  "msg/LoadSet.msg"
  "msg/LoadSpecification.msg"
  "msg/MaxArrayLens.msg"
  "msg/MaxStringLens.msg"
  "msg/Node.msg"
  "msg/NodePosition.msg"
  "msg/NodeState.msg"
  "msg/OptionalParameter.msg"
  "msg/Order.msg"
  "msg/PhysicalParameters.msg"
  "msg/PolygonPoint.msg"
  "msg/Position.msg"
  "msg/ProtocolFeatures.msg"
  "msg/ProtocolLimits.msg"
  "msg/SafetyState.msg"
  "msg/State.msg"
  "msg/Timing.msg"
  "msg/Trajectory.msg"
  "msg/TypeSpecification.msg"
  "msg/Velocity.msg"
  "msg/Visualization.msg"
  "msg/WheelDefinition.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ADD_LINTER_TESTS
)

ament_package()
