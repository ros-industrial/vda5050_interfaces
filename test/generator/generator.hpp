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

#ifndef TEST__GENERATOR__GENERATOR_HPP_
#define TEST__GENERATOR__GENERATOR_HPP_

#include <limits>
#include <random>
#include <string>
#include <vector>

#include "vda5050_msgs/msg/action.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include "vda5050_msgs/msg/action_parameter_value.hpp"
#include "vda5050_msgs/msg/action_state.hpp"
#include "vda5050_msgs/msg/agv_action.hpp"
#include "vda5050_msgs/msg/agv_geometry.hpp"
#include "vda5050_msgs/msg/agv_position.hpp"
#include "vda5050_msgs/msg/battery_state.hpp"
#include "vda5050_msgs/msg/bounding_box_reference.hpp"
#include "vda5050_msgs/msg/connection.hpp"
#include "vda5050_msgs/msg/control_point.hpp"
#include "vda5050_msgs/msg/edge_state.hpp"
#include "vda5050_msgs/msg/envelope2d.hpp"
#include "vda5050_msgs/msg/envelope3d.hpp"
#include "vda5050_msgs/msg/error.hpp"
#include "vda5050_msgs/msg/error_reference.hpp"
#include "vda5050_msgs/msg/factsheet.hpp"
#include "vda5050_msgs/msg/factsheet_action_parameter.hpp"
#include "vda5050_msgs/msg/header.hpp"
#include "vda5050_msgs/msg/info.hpp"
#include "vda5050_msgs/msg/info_reference.hpp"
#include "vda5050_msgs/msg/instant_actions.hpp"
#include "vda5050_msgs/msg/load.hpp"
#include "vda5050_msgs/msg/load_dimensions.hpp"
#include "vda5050_msgs/msg/load_set.hpp"
#include "vda5050_msgs/msg/load_specification.hpp"
#include "vda5050_msgs/msg/max_array_lens.hpp"
#include "vda5050_msgs/msg/max_string_lens.hpp"
#include "vda5050_msgs/msg/network.hpp"
#include "vda5050_msgs/msg/node_position.hpp"
#include "vda5050_msgs/msg/node_state.hpp"
#include "vda5050_msgs/msg/optional_parameters.hpp"
#include "vda5050_msgs/msg/physical_parameters.hpp"
#include "vda5050_msgs/msg/polygon_point.hpp"
#include "vda5050_msgs/msg/position.hpp"
#include "vda5050_msgs/msg/protocol_features.hpp"
#include "vda5050_msgs/msg/protocol_limits.hpp"
#include "vda5050_msgs/msg/safety_state.hpp"
#include "vda5050_msgs/msg/state.hpp"
#include "vda5050_msgs/msg/timing.hpp"
#include "vda5050_msgs/msg/trajectory.hpp"
#include "vda5050_msgs/msg/type_specification.hpp"
#include "vda5050_msgs/msg/vehicle_config.hpp"
#include "vda5050_msgs/msg/velocity.hpp"
#include "vda5050_msgs/msg/version_info.hpp"
#include "vda5050_msgs/msg/visualization.hpp"
#include "vda5050_msgs/msg/wheel_definition.hpp"

using vda5050_msgs::msg::Action;
using vda5050_msgs::msg::ActionParameter;
using vda5050_msgs::msg::ActionParameterValue;
using vda5050_msgs::msg::ActionState;
using vda5050_msgs::msg::AgvAction;
using vda5050_msgs::msg::AgvGeometry;
using vda5050_msgs::msg::AGVPosition;
using vda5050_msgs::msg::BatteryState;
using vda5050_msgs::msg::BoundingBoxReference;
using vda5050_msgs::msg::Connection;
using vda5050_msgs::msg::ControlPoint;
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
using vda5050_msgs::msg::NodePosition;
using vda5050_msgs::msg::NodeState;
using vda5050_msgs::msg::OptionalParameters;
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

/// \brief Utility class to generate random instances of VDA 5050 message types
class RandomDataGenerator
{
public:
  /// \brief Default constructor using a non-deterministic seed
  RandomDataGenerator()
  : rng_(std::random_device()()),
    uint_dist_(0, std::numeric_limits<uint32_t>::max()),
    int_dist_(0, 100),
    float_dist_(
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max()),
    bool_dist_(0, 1),
    string_length_dist_(0, 50),
    milliseconds_dist_(0, 4000000000000L),
    percentage_dist_(0, 100),
    size_dist_(0, 10)
  {
    // Nothing to do
  }

  /// \brief Constructor with a fixed seed for deterministic results
  explicit RandomDataGenerator(uint32_t seed)
  : rng_(seed),
    uint_dist_(0, std::numeric_limits<uint32_t>::max()),
    int_dist_(0, 100),
    float_dist_(
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max()),
    bool_dist_(0, 1),
    string_length_dist_(0, 50),
    milliseconds_dist_(0, 4000000000000L),
    percentage_dist_(0, 100),
    size_dist_(0, 10)
  {
    // Nothing to do
  }

  /// \brief Generate a random unsigned 32-bit integer
  uint32_t generate_random_uint()
  {
    return uint_dist_(rng_);
  }

  /// \brief Generate a random signed 8-bit integer
  int8_t generate_random_int()
  {
    return int_dist_(rng_);
  }

  /// \brief Generate a random 64-bit floating-point number
  double generate_random_float()
  {
    return float_dist_(rng_);
  }

  /// \brief Generate a random index for enum selection
  uint8_t generate_random_index(size_t size)
  {
    std::uniform_int_distribution<uint8_t> index_dist(0, size - 1);
    return index_dist(rng_);
  }

  /// \brief Generate a random boolean value
  bool generate_random_bool()
  {
    return bool_dist_(rng_);
  }

  /// \brief Generate a random alphanumerical string with length upto 50
  std::string generate_random_string()
  {
    int length = string_length_dist_(rng_);
    std::string s;
    s.reserve(length);

    const std::string charset =
      "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

    std::uniform_int_distribution<size_t> char_index_dist(
      0, charset.length() - 1);

    for (int i = 0; i < length; i++)
    {
      s += charset[char_index_dist(rng_)];
    }
    return s;
  }

  /// \brief Generate a random millisecond timestamp
  int64_t generate_milliseconds()
  {
    return milliseconds_dist_(rng_);
  }

  /// \brief Generate a random 64-bit floating-point number
  double generate_random_float()
  {
    return float_dist_(rng_);
    
  /// \brief Generate a random percentage value
  double generate_random_percentage()
  {
    return percentage_dist_(rng_);
  }

  /// \brief Generate a random value for vector length
  uint8_t generate_random_size()
  {
    return size_dist_(rng_);
  }

  /// \brief Generate a random index for enum selection
  uint8_t generate_random_index(size_t size)
  {
    std::uniform_int_distribution<uint8_t> index_dist(0, size - 1);
    return index_dist(rng_);
  }

  /// \brief Generate a random boolean value
  bool generate_random_bool()
  {
    return bool_dist_(rng_);
  }

  /// \brief Generate a random connectionState value
  std::string generate_connection_state()
  {
    std::vector<std::string> states = {
      Connection::ONLINE, Connection::OFFLINE, Connection::CONNECTIONBROKEN};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random actionStatus value
  std::string generate_action_status()
  {
    std::vector<std::string> states = {
      ActionState::ACTION_STATUS_WAITING,
      ActionState::ACTION_STATUS_INITIALIZING,
      ActionState::ACTION_STATUS_RUNNING, ActionState::ACTION_STATUS_PAUSED,
      ActionState::ACTION_STATUS_FINISHED};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random errorLevel value
  std::string generate_random_error_level()
  {
    std::vector<std::string> states = {
      Error::ERROR_LEVEL_WARNING, Error::ERROR_LEVEL_FATAL};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random infoLevel value
  std::string generate_random_info_level()
  {
    std::vector<std::string> states = {
      Info::INFO_LEVEL_INFO, Info::INFO_LEVEL_DEBUG};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random eStop value
  std::string generate_random_e_stop()
  {
    std::vector<std::string> states = {
      SafetyState::E_STOP_AUTOACK, SafetyState::E_STOP_MANUAL,
      SafetyState::E_STOP_REMOTE, SafetyState::E_STOP_NONE};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random operatingMode value
  std::string generate_random_operating_mode()
  {
    std::vector<std::string> states = {
      State::OPERATING_MODE_AUTOMATIC, State::OPERATING_MODE_SEMIAUTOMATIC,
      State::OPERATING_MODE_MANUAL, State::OPERATING_MODE_SERVICE,
      State::OPERATING_MODE_TEACHIN};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random vector of type float64
  std::vector<double> generate_random_float_vector(const uint8_t size)
  {
    std::vector<double> vec(size);
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
      *it = generate_random_float();
    }
    return vec;
  }

  /// \brief Generate a random vector of type T
  template <typename T>
  std::vector<T> generate_random_vector(const uint8_t size)
  {
    std::vector<T> vec(size);
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
      *it = generate<T>();
    }
    return vec;
  }

  /// \brief Generte a random blocking type value
  std::string generate_random_blocking_type()
  {
    std::vector<std::string> states = {
      Action::BLOCKING_TYPE_NONE, Action::BLOCKING_TYPE_SOFT,
      Action::BLOCKING_TYPE_HARD};

    auto state_idx = generate_random_index(states.size());

    return states[state_idx];
  }

  /// \brief Generate a random ActionParameterValue type
  uint8_t generate_random_action_parameter_value_type()
  {
    std::vector<uint8_t> states = {
      ActionParameterValue::TYPE_ARRAY, ActionParameterValue::TYPE_BOOL,
      ActionParameterValue::TYPE_NUMBER, ActionParameterValue::TYPE_STRING,
      ActionParameterValue::TYPE_OBJECT};

    auto state_idx = generate_random_index(states.size());

    return states[state_idx];
  }

  std::string generate_random_scope()
  {
    std::vector<std::string> states = {
      AgvAction::ACTION_SCOPES_EDGE, AgvAction::ACTION_SCOPES_INSTANT,
      AgvAction::ACTION_SCOPES_NODE};

    auto state_idx = generate_random_index(states.size());

    return states[state_idx];
  }

  std::vector<std::string> generate_random_action_scopes()
  {
    std::vector<std::string> action_scopes(generate_random_size());
    for (auto it = action_scopes.begin(); it != action_scopes.end(); ++it)
    {
      *it = generate_random_scope();
    }
    return action_scopes;
  }

  std::string generate_random_agv_action_blocking_type()
  {
    std::vector<std::string> states = {
      AgvAction::BLOCKING_TYPE_SOFT, AgvAction::BLOCKING_TYPES_HARD,
      AgvAction::BLOCKING_TYPES_NONE};

    auto state_idx = generate_random_index(states.size());

    return states[state_idx];
  }

  std::vector<std::string> generate_random_agv_action_blocking_types()
  {
    std::vector<std::string> agv_action_blocking_types(generate_random_size());
    for (auto it = agv_action_blocking_types.begin();
         it != agv_action_blocking_types.end(); ++it)
    {
      *it = generate_random_agv_action_blocking_type();
    }
    return agv_action_blocking_types;
  }

  /// \brief Generate a random vector of type T
  template <typename T>
  std::vector<T> generate_random_vector(const uint8_t size)
  {
    std::vector<T> vec(size);
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
      *it = generate<T>();
    }
    return vec;
  }

  std::string generate_random_value_data_type()
  {
    std::vector<std::string> states = {
      FactsheetActionParameter::VALUE_DATA_TYPE_ARRAY,
      FactsheetActionParameter::VALUE_DATA_TYPE_BOOL,
      FactsheetActionParameter::VALUE_DATA_TYPE_FLOAT,
      FactsheetActionParameter::VALUE_DATA_TYPE_INTEGER,
      FactsheetActionParameter::VALUE_DATA_TYPE_NUMBER,
      FactsheetActionParameter::VALUE_DATA_TYPE_OBJECT};

    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  std::vector<std::string> generate_random_string_vector(const uint8_t size)
  {
    std::vector<std::string> vec(size);
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
      *it = generate_random_string();
    }
    return vec;
  }

  std::string generate_random_support()
  {
    std::vector<std::string> states = {
      OptionalParameters::SUPPORT_REQUIRED,
      OptionalParameters::SUPPORT_SUPPORTED};

    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  std::string generate_random_agv_kinematic()
  {
    std::vector<std::string> states = {
      TypeSpecification::AGV_KINEMATIC_OMNI,
      TypeSpecification::AGV_KINEMATIC_DIFF,
      TypeSpecification::AGV_KINEMATIC_THREEWHEEL};

    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  std::string generate_random_agv_class()
  {
    std::vector<std::string> states = {
      TypeSpecification::AGV_CLASS_FORKLIFT,
      TypeSpecification::AGV_CLASS_CONVEYOR,
      TypeSpecification::AGV_CLASS_TUGGER,
      TypeSpecification::AGV_CLASS_CARRIER};

    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  std::string generate_random_wheel_definition_type()
  {
    std::vector<std::string> states = {
      WheelDefinition::TYPE_DRIVE, WheelDefinition::TYPE_CASTER,
      WheelDefinition::TYPE_FIXED, WheelDefinition::TYPE_MECANUM};

    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a fully populated message of a supported type
  template <typename T>
  T generate()
  {
    T msg;
    if constexpr (std::is_same_v<T, ActionState>)
    {
      msg.action_id = generate_random_string();
      msg.action_type.push_back(generate_random_string());
      msg.action_description.push_back(generate_random_string());
      msg.action_status = generate_action_status();
      msg.result_description.push_back(generate_random_string());
    }
    else if constexpr (std::is_same_v<T, AGVPosition>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.theta = generate_random_float();
      msg.map_id = generate_random_string();
      msg.map_description.push_back(generate_random_string());
      msg.position_initialized = generate_random_bool();
      msg.localization_score.push_back(generate_random_float());
      msg.deviation_range.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, BatteryState>)
    {
      msg.battery_charge = generate_random_percentage();
      msg.battery_voltage.push_back(generate_random_float());
      msg.battery_health.push_back(generate_random_int());
      msg.charging = generate_random_bool();
      msg.reach.push_back(generate_random_uint());
    }
    else if constexpr (std::is_same_v<T, BoundingBoxReference>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.z = generate_random_float();
      msg.theta.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, Connection>)
    {
      msg.header = generate<Header>();
      msg.connection_state = generate_connection_state();
    }
    else if constexpr (std::is_same_v<T, ControlPoint>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.weight = 1.0;
    }
    else if constexpr (std::is_same_v<T, EdgeState>)
    {
      msg.edge_id = generate_random_string();
      msg.sequence_id = generate_random_uint();
      msg.edge_description.push_back(generate_random_string());
      msg.released = generate_random_bool();
      msg.trajectory.push_back(generate<Trajectory>());
    }
    else if constexpr (std::is_same_v<T, Error>)
    {
      msg.error_type = generate_random_string();
      msg.error_references = generate_random_vector<ErrorReference>(10);
      msg.error_description.push_back(generate_random_string());
      msg.error_level = generate_random_error_level();
    }
    else if constexpr (std::is_same_v<T, ErrorReference>)
    {
      msg.reference_key = generate_random_string();
      msg.reference_value = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Header>)
    {
      msg.header_id = generate_random_uint();
      msg.timestamp = generate_milliseconds();
      msg.version = "2.0.0";  // Fix the VDA 5050 version to 2.0.0
      msg.manufacturer = generate_random_string();
      msg.serial_number = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Info>)
    {
      msg.info_type = generate_random_string();
      msg.info_references = generate_random_vector<InfoReference>(5);
      msg.info_description.push_back(generate_random_string());
      msg.info_level = generate_random_info_level();
    }
    else if constexpr (std::is_same_v<T, InfoReference>)
    {
      msg.reference_key = generate_random_string();
      msg.reference_value = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Load>)
    {
      msg.load_id.push_back(generate_random_string());
      msg.load_type.push_back(generate_random_string());
      msg.load_position.push_back(generate_random_string());
      msg.bounding_box_reference.push_back(generate<BoundingBoxReference>());
      msg.load_dimensions.push_back(generate<LoadDimensions>());
      msg.weight.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, LoadDimensions>)
    {
      msg.length = generate_random_float();
      msg.width = generate_random_float();
      msg.height.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, NodePosition>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.theta.push_back(generate_random_float());
      msg.allowed_deviation_x_y.push_back(generate_random_float());
      msg.allowed_deviation_theta.push_back(generate_random_float());
      msg.map_id = generate_random_string();
      msg.map_description.push_back(generate_random_string());
    }
    else if constexpr (std::is_same_v<T, NodeState>)
    {
      msg.node_id = generate_random_string();
      msg.sequence_id = generate_random_uint();
      msg.node_description.push_back(generate_random_string());
      msg.node_position.push_back(generate<NodePosition>());
      msg.released = generate_random_bool();
    }
    else if constexpr (std::is_same_v<T, SafetyState>)
    {
      msg.e_stop = generate_random_e_stop();
      msg.field_violation = generate_random_bool();
    }
    else if constexpr (std::is_same_v<T, State>)
    {
      msg.header = generate<Header>();
      msg.connection_state = generate_connection_state();
      msg.order_id = generate_random_string();
      msg.order_update_id = generate_random_uint();
      msg.zone_set_id.push_back(generate_random_string());
      msg.last_node_id = generate_random_string();
      msg.last_node_sequence_id = generate_random_uint();
      msg.driving = generate_random_bool();
      msg.paused.push_back(generate_random_bool());
      msg.new_base_request.push_back(generate_random_bool());
      msg.distance_since_last_node.push_back(generate_random_float());
      msg.operating_mode = generate_random_operating_mode();
      msg.node_states =
        generate_random_vector<NodeState>(generate_random_size());
      msg.edge_states =
        generate_random_vector<EdgeState>(generate_random_size());
      msg.agv_position.push_back(generate<AGVPosition>());
      msg.velocity.push_back(generate<Velocity>());
      msg.loads = generate_random_vector<Load>(generate_random_size());
      msg.action_states =
        generate_random_vector<ActionState>(generate_random_size());
      msg.battery_state = generate<BatteryState>();
      msg.errors = generate_random_vector<Error>(generate_random_size());
      msg.information = generate_random_vector<Info>(generate_random_size());
      msg.safety_state = generate<SafetyState>();
    }
    else if constexpr (std::is_same_v<T, AgvAction>)
    {
      msg.action_type = generate_random_string();
      msg.action_scopes = generate_random_action_scopes();
      msg.factsheet_action_parameters =
        generate_random_vector<FactsheetActionParameter>(
          generate_random_size());
      msg.result_description.push_back(generate_random_string());
      msg.action_description.push_back(generate_random_string());
      msg.blocking_types = generate_random_agv_action_blocking_types();
    }
    else if constexpr (std::is_same_v<T, AgvGeometry>)
    {
      msg.wheel_definitions =
        generate_random_vector<WheelDefinition>(generate_random_size());
      msg.envelopes_2d =
        generate_random_vector<Envelope2d>(generate_random_size());
      msg.envelopes_3d =
        generate_random_vector<Envelope3d>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, BoundingBoxReference>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.z = generate_random_float();
      msg.theta.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, Envelope2d>)
    {
      msg.set = generate_random_string();
      msg.polygon_points =
        generate_random_vector<PolygonPoint>(generate_random_size());
      msg.description.push_back(generate_random_string());
    }
    else if constexpr (std::is_same_v<T, Envelope3d>)
    {
      msg.set = generate_random_string();
      msg.format = generate_random_string();
      msg.data.push_back(generate_random_string());
      msg.url.push_back(generate_random_string());
      msg.description.push_back(generate_random_string());
    }
    else if constexpr (std::is_same_v<T, Factsheet>)
    {
      msg.header = generate<Header>();
      msg.type_specification = generate<TypeSpecification>();
      msg.physical_parameters = generate<PhysicalParameters>();
      msg.protocol_limits = generate<ProtocolLimits>();
      msg.protocol_features = generate<ProtocolFeatures>();
      msg.agv_geometry = generate<AgvGeometry>();
      msg.load_specification = generate<LoadSpecification>();
      msg.vehicle_config = generate<VehicleConfig>();
    }
    else if constexpr (std::is_same_v<T, FactsheetActionParameter>)
    {
      msg.key = generate_random_string();
      msg.value_data_type = generate_random_value_data_type();
      msg.description.push_back(generate_random_string());
      msg.is_optional.push_back(generate_random_bool());
    }
    else if constexpr (std::is_same_v<T, LoadDimensions>)
    {
      msg.length = generate_random_float();
      msg.width = generate_random_float();
      msg.height.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, LoadSet>)
    {
      msg.set_name = generate_random_string();
      msg.load_type = generate_random_string();
      msg.load_positions =
        generate_random_string_vector(generate_random_size());
      msg.bounding_box_reference.push_back(generate<BoundingBoxReference>());
      msg.load_dimensions.push_back(generate<LoadDimensions>());
      msg.max_weight.push_back(generate_random_float());
      msg.min_load_handling_height.push_back(generate_random_float());
      msg.max_load_handling_height.push_back(generate_random_float());
      msg.min_load_handling_depth.push_back(generate_random_float());
      msg.max_load_handling_depth.push_back(generate_random_float());
      msg.min_load_handling_tilt.push_back(generate_random_float());
      msg.max_load_handling_tilt.push_back(generate_random_float());
      msg.agv_speed_limit.push_back(generate_random_float());
      msg.agv_acceleration_limit.push_back(generate_random_float());
      msg.agv_deceleration_limit.push_back(generate_random_float());
      msg.pick_time.push_back(generate_random_float());
      msg.drop_time.push_back(generate_random_float());
      msg.description.push_back(generate_random_string());
    }
    else if constexpr (std::is_same_v<T, LoadSpecification>)
    {
      msg.load_positions =
        generate_random_string_vector(generate_random_size());
      msg.load_sets = generate_random_vector<LoadSet>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, MaxArrayLens>)
    {
      msg.order_nodes = generate_random_uint();
      msg.order_edges = generate_random_uint();
      msg.node_actions = generate_random_uint();
      msg.edge_actions = generate_random_uint();
      msg.actions_actions_parameters = generate_random_uint();
      msg.instant_actions = generate_random_uint();
      msg.trajectory_knot_vector = generate_random_uint();
      msg.trajectory_control_points = generate_random_uint();
      msg.state_node_states = generate_random_uint();
      msg.state_edge_states = generate_random_uint();
      msg.state_loads = generate_random_uint();
      msg.state_action_states = generate_random_uint();
      msg.state_errors = generate_random_uint();
      msg.state_information = generate_random_uint();
      msg.error_error_references = generate_random_uint();
      msg.information_info_references = generate_random_uint();
    }
    else if constexpr (std::is_same_v<T, MaxStringLens>)
    {
      msg.msg_len.push_back(generate_random_uint());
      msg.topic_serial_len.push_back(generate_random_uint());
      msg.topic_elem_len.push_back(generate_random_uint());
      msg.id_len.push_back(generate_random_uint());
      msg.enum_len.push_back(generate_random_uint());
      msg.load_id_len.push_back(generate_random_uint());
      msg.id_numerical_only.push_back(generate_random_bool());
    }
    else if constexpr (std::is_same_v<T, Network>)
    {
      msg.dns_servers = generate_random_string_vector(generate_random_size());
      msg.ntp_servers = generate_random_string_vector(generate_random_size());
      msg.local_ip_address.push_back(generate_random_string());
      msg.netmask.push_back(generate_random_string());
      msg.default_gateway.push_back(generate_random_string());
    }
    else if constexpr (std::is_same_v<T, OptionalParameters>)
    {
      msg.parameter = generate_random_string();
      msg.support = generate_random_support();
      msg.description.push_back(generate_random_string());
    }
    else if constexpr (std::is_same_v<T, PhysicalParameters>)
    {
      msg.speed_min = generate_random_float();
      msg.speed_max = generate_random_float();
      msg.acceleration_max = generate_random_float();
      msg.deceleration_max = generate_random_float();
      msg.height_min = generate_random_float();
      msg.height_max = generate_random_float();
      msg.width = generate_random_float();
      msg.length = generate_random_float();
      msg.angular_speed_min.push_back(generate_random_float());
      msg.angular_speed_max.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, PolygonPoint>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, Position>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.theta.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, ProtocolFeatures>)
    {
      msg.optional_parameters =
        generate_random_vector<OptionalParameters>(generate_random_size());
      msg.agv_actions =
        generate_random_vector<AgvAction>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, ProtocolLimits>)
    {
      msg.max_string_lens = generate<MaxStringLens>();
      msg.max_array_lens = generate<MaxArrayLens>();
      msg.timing = generate<Timing>();
    }
    else if constexpr (std::is_same_v<T, Timing>)
    {
      msg.min_order_interval = generate_random_float();
      msg.min_state_interval = generate_random_float();
      msg.default_state_interval.push_back(generate_random_float());
      msg.visualization_interval.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, TypeSpecification>)
    {
      msg.series_name = generate_random_string();
      msg.agv_kinematic = generate_random_agv_kinematic();
      msg.agv_class = generate_random_agv_class();
      msg.max_load_mass = generate_random_float();
      msg.localization_types =
        generate_random_string_vector(generate_random_size());
      msg.navigation_types =
        generate_random_string_vector(generate_random_size());
      msg.series_description.push_back(generate_random_string());
    }
    else if constexpr (std::is_same_v<T, VehicleConfig>)
    {
      msg.versions =
        generate_random_vector<VersionInfo>(generate_random_size());
      msg.network.push_back(generate<Network>());
    }
    else if constexpr (std::is_same_v<T, VersionInfo>)
    {
      msg.key = generate_random_string();
      msg.value = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, WheelDefinition>)
    {
      msg.type = generate_random_wheel_definition_type();
      msg.is_active_driven = generate_random_bool();
      msg.is_active_steered = generate_random_bool();
      msg.position = generate<Position>();
      msg.diameter = generate_random_float();
      msg.width = generate_random_float();
      msg.center_displacement = generate_random_float();
      msg.constraints.push_back(generate_random_string());
    }
    else if constexpr (std::is_same_v<T, Trajectory>)
    {
      msg.degree = generate_random_float();
      msg.knot_vector = generate_random_float_vector(generate_random_size());
      msg.control_points =
        generate_random_vector<ControlPoint>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, Velocity>)
    {
      msg.vx.push_back(generate_random_float());
      msg.vy.push_back(generate_random_float());
      msg.omega.push_back(generate_random_float());
    }
    else if constexpr (std::is_same_v<T, Visualization>)
    {
      msg.header = generate<Header>();
      msg.agv_position.push_back(generate<AGVPosition>());
      msg.velocity.push_back(generate<Velocity>());
    }
    else if constexpr (std::is_same_v<T, ActionParameterValue>)
    {
      msg.type = generate_random_action_parameter_value_type();
      msg.value = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, ActionParameter>)
    {
      msg.key = generate_random_string();
      msg.value = generate<ActionParameterValue>();
    }
    else if constexpr (std::is_same_v<T, Action>)
    {
      msg.action_type = generate_random_string();
      msg.action_id = generate_random_string();
      msg.blocking_type = generate_random_blocking_type();
      msg.action_description.push_back(generate_random_string());
      msg.action_parameters =
        generate_random_vector<ActionParameter>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, InstantActions>)
    {
      msg.header = generate<Header>();
      msg.actions = generate_random_vector<Action>(generate_random_size());
    }
    else
    {
      throw std::runtime_error(
        "No random data generator defined for this custom type: " +
        std::string(typeid(T).name()));
    }
    return msg;
  }

private:
  /// \brief Mersenne Twister random number engine
  std::mt19937 rng_;

  /// \brief Distribution for unsigned 32-bit integers
  std::uniform_int_distribution<uint32_t> uint_dist_;

  /// \brief Distribution for signed 8-bit integers
  std::uniform_int_distribution<int8_t> int_dist_;

  /// \brief Distribution for 64-bit floating-point numbers
  std::uniform_real_distribution<double> float_dist_;

  /// \brief Distribution for a boolean value
  std::uniform_int_distribution<int> bool_dist_;

  /// \brief Distribution for random string lengths
  std::uniform_int_distribution<int> string_length_dist_;

  /// \brief Distribution for milliseconds from epoch
  std::uniform_int_distribution<int64_t> milliseconds_dist_;

  /// \brief Distribution for random percentage values
  std::uniform_real_distribution<double> percentage_dist_;

  /// \brief Distribution for random vector size
  std::uniform_int_distribution<uint8_t> size_dist_;
};

#endif  // TEST__GENERATOR__GENERATOR_HPP_
