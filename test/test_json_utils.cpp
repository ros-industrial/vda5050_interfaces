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
#include "vda5050_msgs/json_utils/agv_action.hpp"
#include "vda5050_msgs/json_utils/agv_geometry.hpp"
#include "vda5050_msgs/json_utils/bounding_box_reference.hpp"
#include "vda5050_msgs/json_utils/connection.hpp"
#include "vda5050_msgs/json_utils/envelope2d.hpp"
#include "vda5050_msgs/json_utils/envelope3d.hpp"
#include "vda5050_msgs/json_utils/factsheet.hpp"
#include "vda5050_msgs/json_utils/factsheet_action_parameter.hpp"
#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/json_utils/load_dimensions.hpp"
#include "vda5050_msgs/json_utils/load_set.hpp"
#include "vda5050_msgs/json_utils/load_specification.hpp"
#include "vda5050_msgs/json_utils/max_array_lens.hpp"
#include "vda5050_msgs/json_utils/max_string_lens.hpp"
#include "vda5050_msgs/json_utils/network.hpp"
#include "vda5050_msgs/json_utils/optional_parameters.hpp"
#include "vda5050_msgs/json_utils/physical_parameters.hpp"
#include "vda5050_msgs/json_utils/polygon_point.hpp"
#include "vda5050_msgs/json_utils/position.hpp"
#include "vda5050_msgs/json_utils/protocol_features.hpp"
#include "vda5050_msgs/json_utils/protocol_limits.hpp"
#include "vda5050_msgs/json_utils/timing.hpp"
#include "vda5050_msgs/json_utils/type_specification.hpp"
#include "vda5050_msgs/json_utils/vehicle_config.hpp"
#include "vda5050_msgs/json_utils/version_info.hpp"
#include "vda5050_msgs/json_utils/wheel_definition.hpp"

using vda5050_msgs::msg::AgvAction;
using vda5050_msgs::msg::AgvGeometry;
using vda5050_msgs::msg::BoundingBoxReference;
using vda5050_msgs::msg::Connection;
using vda5050_msgs::msg::Envelope2d;
using vda5050_msgs::msg::Envelope3d;
using vda5050_msgs::msg::Factsheet;
using vda5050_msgs::msg::FactsheetActionParameter;
using vda5050_msgs::msg::Header;
using vda5050_msgs::msg::LoadDimensions;
using vda5050_msgs::msg::LoadSet;
using vda5050_msgs::msg::LoadSpecification;
using vda5050_msgs::msg::MaxArrayLens;
using vda5050_msgs::msg::MaxStringLens;
using vda5050_msgs::msg::Network;
using vda5050_msgs::msg::OptionalParameters;
using vda5050_msgs::msg::PhysicalParameters;
using vda5050_msgs::msg::PolygonPoint;
using vda5050_msgs::msg::Position;
using vda5050_msgs::msg::ProtocolFeatures;
using vda5050_msgs::msg::ProtocolLimits;
using vda5050_msgs::msg::Timing;
using vda5050_msgs::msg::TypeSpecification;
using vda5050_msgs::msg::VehicleConfig;
using vda5050_msgs::msg::VersionInfo;
using vda5050_msgs::msg::WheelDefinition;

// List of types to be tested for serialization round-trip
using SerializableTypes = ::testing::Types<
  AgvAction, AgvGeometry, BoundingBoxReference, Connection, Envelope2d,
  Envelope3d, Factsheet, FactsheetActionParameter, Header, LoadDimensions,
  LoadSet, LoadSpecification, MaxArrayLens, MaxStringLens, Network,
  OptionalParameters, PhysicalParameters, PolygonPoint, Position,
  ProtocolFeatures, ProtocolLimits, Timing, TypeSpecification, VehicleConfig,
  VersionInfo, WheelDefinition>;

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
      << "\nOriginal:\n"
      << to_yaml(original) << "Deserialized:\n"
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
