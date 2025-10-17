/**
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <rosidl_runtime_cpp/traits.hpp>

#include "generator/generator.hpp"
#include "vda5050_msgs/json_utils/action.hpp"
#include "vda5050_msgs/json_utils/action_parameter.hpp"
#include "vda5050_msgs/json_utils/action_parameter_value.hpp"
#include "vda5050_msgs/json_utils/action_state.hpp"
#include "vda5050_msgs/json_utils/agv_action.hpp"
#include "vda5050_msgs/json_utils/agv_geometry.hpp"
#include "vda5050_msgs/json_utils/agv_position.hpp"
#include "vda5050_msgs/json_utils/battery_state.hpp"
#include "vda5050_msgs/json_utils/bounding_box_reference.hpp"
#include "vda5050_msgs/json_utils/connection.hpp"
#include "vda5050_msgs/json_utils/control_point.hpp"
#include "vda5050_msgs/json_utils/edge.hpp"
#include "vda5050_msgs/json_utils/edge_state.hpp"
#include "vda5050_msgs/json_utils/envelope2d.hpp"
#include "vda5050_msgs/json_utils/envelope3d.hpp"
#include "vda5050_msgs/json_utils/error.hpp"
#include "vda5050_msgs/json_utils/error_reference.hpp"
#include "vda5050_msgs/json_utils/factsheet.hpp"
#include "vda5050_msgs/json_utils/factsheet_action_parameter.hpp"
#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/json_utils/info.hpp"
#include "vda5050_msgs/json_utils/info_reference.hpp"
#include "vda5050_msgs/json_utils/instant_actions.hpp"
#include "vda5050_msgs/json_utils/load.hpp"
#include "vda5050_msgs/json_utils/load_dimensions.hpp"
#include "vda5050_msgs/json_utils/load_set.hpp"
#include "vda5050_msgs/json_utils/load_specification.hpp"
#include "vda5050_msgs/json_utils/max_array_lens.hpp"
#include "vda5050_msgs/json_utils/max_string_lens.hpp"
#include "vda5050_msgs/json_utils/network.hpp"
#include "vda5050_msgs/json_utils/node.hpp"
#include "vda5050_msgs/json_utils/node_position.hpp"
#include "vda5050_msgs/json_utils/node_state.hpp"
#include "vda5050_msgs/json_utils/optional_parameters.hpp"
#include "vda5050_msgs/json_utils/order.hpp"
#include "vda5050_msgs/json_utils/physical_parameters.hpp"
#include "vda5050_msgs/json_utils/polygon_point.hpp"
#include "vda5050_msgs/json_utils/position.hpp"
#include "vda5050_msgs/json_utils/protocol_features.hpp"
#include "vda5050_msgs/json_utils/protocol_limits.hpp"
#include "vda5050_msgs/json_utils/safety_state.hpp"
#include "vda5050_msgs/json_utils/state.hpp"
#include "vda5050_msgs/json_utils/timing.hpp"
#include "vda5050_msgs/json_utils/trajectory.hpp"
#include "vda5050_msgs/json_utils/type_specification.hpp"
#include "vda5050_msgs/json_utils/vehicle_config.hpp"
#include "vda5050_msgs/json_utils/velocity.hpp"
#include "vda5050_msgs/json_utils/version_info.hpp"
#include "vda5050_msgs/json_utils/visualization.hpp"
#include "vda5050_msgs/json_utils/wheel_definition.hpp"

using vda5050_msgs::msg::Action;
using vda5050_msgs::msg::ActionParameter;
using vda5050_msgs::msg::ActionParameterValue;
using vda5050_msgs::msg::ActionState;
using vda5050_msgs::msg::AGVAction;
using vda5050_msgs::msg::AGVGeometry;
using vda5050_msgs::msg::AGVPosition;
using vda5050_msgs::msg::BatteryState;
using vda5050_msgs::msg::BoundingBoxReference;
using vda5050_msgs::msg::Connection;
using vda5050_msgs::msg::ControlPoint;
using vda5050_msgs::msg::Edge;
using vda5050_msgs::msg::EdgeState;
using vda5050_msgs::msg::Envelope2d;
using vda5050_msgs::msg::Envelope3d;
using vda5050_msgs::msg::Error;
using vda5050_msgs::msg::ErrorReference;
using vda5050_msgs::msg::Factsheet;
using vda5050_msgs::msg::FactsheetActionParameter;
using vda5050_msgs::msg::Header;
using vda5050_msgs::msg::Info;
using vda5050_msgs::msg::InfoReference;
using vda5050_msgs::msg::InstantActions;
using vda5050_msgs::msg::Load;
using vda5050_msgs::msg::LoadDimensions;
using vda5050_msgs::msg::LoadSet;
using vda5050_msgs::msg::LoadSpecification;
using vda5050_msgs::msg::MaxArrayLens;
using vda5050_msgs::msg::MaxStringLens;
using vda5050_msgs::msg::Network;
using vda5050_msgs::msg::Node;
using vda5050_msgs::msg::NodePosition;
using vda5050_msgs::msg::NodeState;
using vda5050_msgs::msg::OptionalParameters;
using vda5050_msgs::msg::Order;
using vda5050_msgs::msg::PhysicalParameters;
using vda5050_msgs::msg::PolygonPoint;
using vda5050_msgs::msg::Position;
using vda5050_msgs::msg::ProtocolFeatures;
using vda5050_msgs::msg::ProtocolLimits;
using vda5050_msgs::msg::SafetyState;
using vda5050_msgs::msg::State;
using vda5050_msgs::msg::Timing;
using vda5050_msgs::msg::Trajectory;
using vda5050_msgs::msg::TypeSpecification;
using vda5050_msgs::msg::VehicleConfig;
using vda5050_msgs::msg::Velocity;
using vda5050_msgs::msg::VersionInfo;
using vda5050_msgs::msg::Visualization;
using vda5050_msgs::msg::WheelDefinition;

// List of types to be tested for serialization round-trip
using SerializableTypes = ::testing::Types<
  Action, ActionParameter, ActionParameterValue, ActionState, AGVAction,
  AGVGeometry, AGVPosition, BatteryState, BoundingBoxReference, Connection,
  ControlPoint, EdgeState, Envelope2d, Envelope3d, Error, ErrorReference,
  Factsheet, FactsheetActionParameter, Header, Info, InfoReference,
  InstantActions, Load, LoadDimensions, LoadSet, LoadSpecification,
  MaxArrayLens, MaxStringLens, Network, Node, NodePosition, NodeState,
  OptionalParameters, Order, PhysicalParameters, PolygonPoint, Position,
  ProtocolFeatures, ProtocolLimits, SafetyState, State, Timing, Trajectory,
  TypeSpecification, VehicleConfig, Velocity, VersionInfo, Visualization,
  WheelDefinition>;

template <typename T>
class SerializationTest : public ::testing::Test
{
protected:
  // Random data generator instance
  RandomDataGenerator generator;

  /// \brief Performs a serialization round-trip for a given message object
  ///
  /// \param original Object of type T to be tested
  void round_trip_test(const T& original)
  {
    // Serialize the original object into JSON
    nlohmann::json serialized_data = original;

    // Deserialize the JSON object back into an object of type T
    T deserialized_object = serialized_data;

    EXPECT_EQ(original, deserialized_object)
      << "Serialization round-trip failed for type: " << typeid(T).name()
      << "\n\nOriginal:\n"
      << to_yaml(original) << "\nDeserialized:\n"
      << to_yaml(deserialized_object);
  }
};

TYPED_TEST_SUITE(SerializationTest, SerializableTypes);

TYPED_TEST(SerializationTest, RoundTrip)
{
  // Number of iterations for round-trip of each object
  const int num_random_tests = 100;

  for (int i = 0; i < num_random_tests; i++)
  {
    SCOPED_TRACE("Test iteration " + std::to_string(i));
    TypeParam random_object = this->generator.template generate<TypeParam>();
    this->round_trip_test(random_object);
  }
}
