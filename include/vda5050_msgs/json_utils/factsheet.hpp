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

#ifndef VDA5050_MSGS__JSON_UTILS__FACTSHEET_HPP_
#define VDA5050_MSGS__JSON_UTILS__FACTSHEET_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/msg/factsheet.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::FactsheetActionParameter object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize value_data_type
void to_json(nlohmann::json& j, const FactsheetActionParameter& msg)
{
  j["key"] = msg.key;

  if (
    msg.value_data_type == "BOOL" || msg.value_data_type == "NUMBER" ||
    msg.value_data_type == "INTEGER" || msg.value_data_type == "FLOAT" ||
    msg.value_data_type == "OBJECT" || msg.value_data_type == "ARRAY")
  {
    j["valueDataType"] = msg.value_data_type;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected value_data_type in "
      "factsheet_action_parameter field");
  }

  if (!msg.description.empty())
  {
    j["description"] = msg.description;
  }

  if (!msg.is_optional.empty())
  {
    j["isOptional"] = msg.is_optional;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::FactsheetActionParameter object
///
/// \param j Reference to the JSON object containing serialized FactsheetActionParameter data
/// \param msg Reference to the FactsheetActionParameter message to populate
///
/// \throws std::runtime_error If failed to deserialize value_data_type
void from_json(const nlohmann::json& j, FactsheetActionParameter& msg)
{
  auto key = j.at("key").get<std::string>();
  msg.key = key;

  auto value_data_type = j.at("valueDataType").get<std::string>();
  if (
    value_data_type == "BOOL" || value_data_type == "NUMBER" ||
    value_data_type == "INTEGER" || value_data_type == "FLOAT" ||
    value_data_type == "OBJECT" || value_data_type == "ARRAY")
  {
    msg.value_data_type = value_data_type;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected value_data_type in "
      "factsheet_action_parameter field");
  }

  if (j.contains("description"))
  {
    msg.description.push_back(j.at("description").get<std::string>());
  }

  if (j.contains("isOptional"))
  {
    msg.is_optional.push_back(j.at("isOptional").get<bool>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::AgvAction object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize action_scopes or blocking_types
void to_json(nlohmann::json& j, const AgvAction& msg)
{
  j["actionType"] = msg.action_type;

  for (std::string scope : msg.action_scopes)
  {
    if (scope != "INSTANT" && scope != "NODE" && scope != "EDGE")
    {
      throw std::runtime_error(
        "Serialization error: Unexpected scope in action_scopes field");
    }
  }
  j["actionScopes"] = msg.action_scopes;

  if (!msg.factsheet_action_parameters.empty())
  {
    j["factsheetActionParameters"] = msg.factsheet_action_parameters;
  }

  if (!msg.result_description.empty())
  {
    j["resultDescription"] = msg.result_description;
  }

  if (!msg.action_description.empty())
  {
    j["actionDescription"] = msg.action_description;
  }

  if (!msg.blocking_types.empty())
  {
    for (std::string type : msg.blocking_types)
    {
      if (type != "NONE" && type != "SOFT" && type != "HARD")
      {
        throw std::runtime_error(
          "Serialization error: Unexpected type in blocking_types field");
      }
    }

    j["blockingTypes"] = msg.blocking_types;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::AgvAction object
///
/// \param j Reference to the JSON object containing serialized AgvAction data
/// \param msg Reference to the AgvAction message to populate
///
/// \throws std::runtime_error If failed to deserialize action_scopes or blocking_types
void from_json(const nlohmann::json& j, AgvAction& msg)
{
  auto action_type = j.at("actionType").get<std::string>();
  msg.action_type = action_type;

  auto action_scopes = j.at("actionScopes").get<std::vector<std::string>>();
  for (std::string scope : action_scopes)
  {
    if (scope != "INSTANT" && scope != "NODE" && scope != "EDGE")
    {
      throw std::runtime_error(
        "Serialization error: Unexpected scope in action_scopes field");
    }
  }
  msg.action_scopes = action_scopes;

  if (j.contains("factsheetActionParameters"))
  {
    msg.factsheet_action_parameters =
      j.at("factsheetActionParameters")
        .get<std::vector<FactsheetActionParameter>>();
  }

  if (j.contains("resultDescription"))
  {
    msg.result_description.push_back(
      j.at("resultDescription").get<std::string>());
  }

  if (j.contains("actionDescription"))
  {
    msg.action_description.push_back(
      j.at("actionDescription").get<std::string>());
  }

  if (j.contains("blockingTypes"))
  {
    auto blocking_types = j.at("blockingTypes").get<std::vector<std::string>>();
    for (std::string type : blocking_types)
    {
      if (type != "NONE" && type != "SOFT" && type != "HARD")
      {
        throw std::runtime_error(
          "Serialization error: Unexpected type in blocking_types field");
      }
    }
    msg.blocking_types = blocking_types;
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Position object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Position message object to serialize
void to_json(nlohmann::json& j, const Position& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;

  if (!msg.theta.empty())
  {
    j["theta"] = msg.theta;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Position object
///
/// \param j Reference to the JSON object containing serialized Position data
/// \param msg Reference to the Position message to populate
void from_json(const nlohmann::json& j, Position& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();

  if (j.contains("theta"))
  {
    msg.theta.push_back(j.at("theta").get<double>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::WheelDefinition object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the WheelDefinition message object to serialize
///
/// \throws std::runtime_error If failed to serialize type field
void to_json(nlohmann::json& j, const WheelDefinition& msg)
{
  if (
    msg.type == "DRIVE" || msg.type == "CASTER" || msg.type == "FIXED" ||
    msg.type == "MECANUM")
  {
    j["type"] = msg.type;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected type in wheel_definition field");
  }

  j["isActiveDriven"] = msg.is_active_driven;
  j["isActiveSteered"] = msg.is_active_steered;
  j["position"] = msg.position;
  j["diameter"] = msg.diameter;
  j["width"] = msg.width;
  j["centerDisplacement"] = msg.center_displacement;

  if (!msg.constraints.empty())
  {
    j["constraints"] = msg.constraints;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::WheelDefinition object
///
/// \param j Reference to the JSON object containing serialized WheelDefinition data
/// \param msg Reference to the WheelDefinition message to populate
///
/// \throws std::runtime_error If failed to deserialize type field
void from_json(const nlohmann::json& j, WheelDefinition& msg)
{
  auto type = j.at("type").get<std::string>();
  if (
    type == "DRIVE" || type == "CASTER" || type == "FIXED" || type == "MECANUM")
  {
    msg.type = type;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected type in wheel_definition field");
  }

  msg.is_active_driven = j.at("isActiveDriven").get<bool>();
  msg.is_active_steered = j.at("isActiveSteered").get<bool>();

  auto position = j.at("position").get<Position>();
  msg.position = position;

  msg.diameter = j.at("diameter").get<double>();
  msg.width = j.at("width").get<double>();
  msg.center_displacement.push_back(j.at("centerDisplacement").get<double>());

  if (j.contains("constraints"))
  {
    msg.constraints.push_back(j.at("constraints").get<std::string>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::PolygonPoint object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the PolygonPoint message object to serialize
void to_json(nlohmann::json& j, const PolygonPoint& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::PolygonPoint object
///
/// \param j Reference to the JSON object containing serialized PolygonPoint data
/// \param msg Reference to the PolygonPoint message to populate
void from_json(const nlohmann::json& j, PolygonPoint& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Envelope2d object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Envelope2d message object to serialize
void to_json(nlohmann::json& j, const Envelope2d& msg)
{
  j["set"] = msg.set;
  j["polygonPoints"] = msg.polygon_points;

  if (!msg.description.empty())
  {
    j["description"] = msg.description;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Envelope2d object
///
///\param j Reference to the JSON object containing serialized Envelope2d data
/// \param msg Reference to the Envelope2d message to populate
void from_json(const nlohmann::json& j, Envelope2d& msg)
{
  msg.set = j.at("set").get<std::string>();
  msg.polygon_points = j.at("polygonPoints").get<std::vector<PolygonPoint>>();

  if (j.contains("description"))
  {
    msg.description.push_back(j.at("description").get<std::string>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Envelope3d object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Envelope3d message object to serialize
void to_json(nlohmann::json& j, const Envelope3d& msg)
{
  j["set"] = msg.set;
  j["format"] = msg.format;

  if (!msg.data.empty())
  {
    j["data"] = msg.data;
  }

  if (!msg.url.empty())
  {
    j["url"] = msg.url;
  }

  if (!msg.description.empty())
  {
    j["description"] = msg.description;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Envelope3d object
///
/// \param j Reference to the JSON object containing serialized Envelope3d data
/// \param msg Reference to the Envelope3d message to populate
void from_json(const nlohmann::json& j, Envelope3d& msg)
{
  msg.set = j.at("set").get<std::string>();
  msg.format = j.at("format").get<std::string>();

  if (j.contains("data"))
  {
    msg.data.push_back(j.at("data").get<std::string>());
  }

  if (j.contains("url"))
  {
    msg.url.push_back(j.at("url").get<std::string>());
  }

  if (j.contains("description"))
  {
    msg.description.push_back(j.at("description").get<std::string>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::AgvGeometry object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the AgvGeometry message object to serialize
void to_json(nlohmann::json& j, const AgvGeometry& msg)
{
  if (!msg.wheel_definitions.empty())
  {
    j["wheelDefinitions"] = msg.wheel_definitions;
  }

  if (!msg.envelopes_2d.empty())
  {
    j["envelopes2d"] = msg.envelopes_2d;
  }

  if (!msg.envelopes_3d.empty())
  {
    j["envelopes3d"] = msg.envelopes_3d;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::AgvGeometry object
///
/// \param j Reference to the JSON object containing serialized AgvGeometry data
/// \param msg Reference to the AgvGeometry message to populate
void from_json(const nlohmann::json& j, AgvGeometry& msg)
{
  if (j.contains("wheelDefinitions"))
  {
    msg.wheel_definitions =
      j.at("wheelDefinitions").get<std::vector<WheelDefinition>>();
  }

  if (j.contains("envelopes2d"))
  {
    msg.envelopes_2d = j.at("envelopes2d").get<std::vector<Envelope2d>>();
  }

  if (j.contains("envelopes3d"))
  {
    msg.envelopes_3d = j.at("envelopes3d").get<std::vector<Envelope3d>>();
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::BoundingBoxReference object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const BoundingBoxReference& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["z"] = msg.z;

  if (!msg.theta.empty())
  {
    j["theta"] = msg.theta.front();
  }
}

// ============================================================================
/// \brief Populate a vda5050_msgs::msg::BoundingBoxReference object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, BoundingBoxReference& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.z = j.at("z").get<double>();

  if (j.contains("theta"))
  {
    msg.theta.push_back(j.at("theta").get<double>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::LoadDimensions object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const LoadDimensions& msg)
{
  j["length"] = msg.length;
  j["width"] = msg.width;

  if (!msg.height.empty())
  {
    j["height"] = msg.height.front();
  }
}

// ============================================================================
/// \brief Populate a vda5050_msgs::msg::LoadDimensions object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, LoadDimensions& msg)
{
  msg.length = j.at("length").get<double>();
  msg.width = j.at("width").get<double>();

  if (j.contains("height"))
  {
    msg.height.push_back(j.at("height").get<double>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::LoadSet object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the LoadSet message object to serialize
void to_json(nlohmann::json& j, const LoadSet& msg)
{
  j["setName"] = msg.set_name;
  j["loadType"] = msg.load_type;

  if (!msg.load_positions.empty())
  {
    j["loadPositions"] = msg.load_positions;
  }

  if (!msg.bounding_box_reference.empty())
  {
    j["boundingBoxReference"] = msg.bounding_box_reference;
  }

  if (!msg.load_dimensions.empty())
  {
    j["loadDimensions"] = msg.load_dimensions;
  }

  if (!msg.max_weight.empty())
  {
    j["maxWeight"] = msg.max_weight;
  }

  if (!msg.min_load_handling_height.empty())
  {
    j["minLoadHandlingHeight"] = msg.min_load_handling_height;
  }

  if (!msg.max_load_handling_height.empty())
  {
    j["maxLoadHandlingHeight"] = msg.max_load_handling_height;
  }

  if (!msg.min_load_handling_depth.empty())
  {
    j["minLoadHandlingDepth"] = msg.min_load_handling_depth;
  }

  if (!msg.max_load_handling_depth.empty())
  {
    j["maxLoadHandlingDepth"] = msg.max_load_handling_depth;
  }

  if (!msg.min_load_handling_tilt.empty())
  {
    j["minLoadHandlingTilt"] = msg.min_load_handling_tilt;
  }

  if (!msg.max_load_handling_tilt.empty())
  {
    j["maxLoadHandlingTilt"] = msg.max_load_handling_tilt;
  }

  if (!msg.agv_speed_limit.empty())
  {
    j["agvSpeedLimit"] = msg.agv_speed_limit;
  }

  if (!msg.agv_acceleration_limit.empty())
  {
    j["agvAccelerationLimit"] = msg.agv_acceleration_limit;
  }

  if (!msg.agv_deceleration_limit.empty())
  {
    j["agvDecelerationLimit"] = msg.agv_deceleration_limit;
  }

  if (!msg.pick_time.empty())
  {
    j["pickTime"] = msg.pick_time;
  }

  if (!msg.drop_time.empty())
  {
    j["dropTime"] = msg.drop_time;
  }

  if (!msg.description.empty())
  {
    j["description"] = msg.description;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::LoadSet object
///
/// \param j Reference to the JSON object containing serialized LoadSet data
/// \param msg Reference to the LoadSet message to populate
void from_json(const nlohmann::json& j, LoadSet& msg)
{
  auto set_name = j.at("setName").get<std::string>();
  msg.set_name = set_name;

  auto load_type = j.at("loadType").get<std::string>();
  msg.load_type = load_type;

  if (j.contains("loadPositions"))
  {
    msg.load_positions = j.at("loadPositions").get<std::vector<std::string>>();
  }

  if (j.contains("boundingBoxReference"))
  {
    msg.bounding_box_reference.push_back(
      j.at("boundingBoxReference").get<BoundingBoxReference>());
  }

  if (j.contains("loadDimensions"))
  {
    msg.load_dimensions.push_back(j.at("loadDimensions").get<LoadDimensions>());
  }

  if (j.contains("maxWeight"))
  {
    msg.max_weight.push_back(j.at("maxWeight").get<double>());
  }

  if (j.contains("minLoadHandlingHeight"))
  {
    msg.min_load_handling_height.push_back(
      j.at("minLoadHandlingHeight").get<double>());
  }

  if (j.contains("maxLoadHandlingHeight"))
  {
    msg.max_load_handling_height.push_back(
      j.at("maxLoadHandlingHeight").get<double>());
  }

  if (j.contains("minLoadHandlingDepth"))
  {
    msg.min_load_handling_depth.push_back(
      j.at("minLoadHandlingDepth").get<double>());
  }

  if (j.contains("maxLoadHandlingDepth"))
  {
    msg.max_load_handling_depth.push_back(
      j.at("maxLoadHandlingDepth").get<double>());
  }

  if (j.contains("minLoadHandlingTilt"))
  {
    msg.min_load_handling_tilt.push_back(
      j.at("minLoadHandlingTilt").get<double>());
  }

  if (j.contains("maxLoadHandlingTilt"))
  {
    msg.max_load_handling_tilt.push_back(
      j.at("maxLoadHandlingTilt").get<double>());
  }

  if (j.contains("agvSpeedLimit"))
  {
    msg.agv_speed_limit.push_back(j.at("agvSpeedLimit").get<double>());
  }

  if (j.contains("agvAccelerationLimit"))
  {
    msg.agv_acceleration_limit.push_back(
      j.at("agvAccelerationLimit").get<double>());
  }

  if (j.contains("agvDecelerationLimit"))
  {
    msg.agv_deceleration_limit.push_back(
      j.at("agvDecelerationLimit").get<double>());
  }

  if (j.contains("pickTime"))
  {
    msg.pick_time.push_back(j.at("pickTime").get<double>());
  }

  if (j.contains("dropTime"))
  {
    msg.drop_time.push_back(j.at("dropTime").get<double>());
  }

  if (j.contains("description"))
  {
    msg.description.push_back(j.at("description").get<std::string>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::LoadSpecification object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the LoadSpecification message object to serialize
void to_json(nlohmann::json& j, const LoadSpecification& msg)
{
  if (!msg.load_positions.empty())
  {
    j["loadPositions"] = msg.load_positions;
  }

  if (!msg.load_sets.empty())
  {
    j["loadSets"] = msg.load_sets;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::LoadSpecification object
///
/// \param j Reference to the JSON object containing serialized LoadSpecification data
/// \param msg Reference to the LoadSpecification message to populate
void from_json(const nlohmann::json& j, LoadSpecification& msg)
{
  if (j.contains("loadPositions"))
  {
    msg.load_positions = j.at("loadPositions").get<std::vector<std::string>>();
  }

  if (j.contains("loadSets"))
  {
    msg.load_sets = j.at("loadSets").get<std::vector<LoadSet>>();
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::MaxArrayLens object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the MaxArrayLens message object to serialize
void to_json(nlohmann::json& j, const MaxArrayLens& msg)
{
  j["orderNodes"] = msg.order_nodes;
  j["orderEdges"] = msg.order_edges;
  j["nodeActions"] = msg.node_actions;
  j["edgeActions"] = msg.edge_actions;
  j["actionsActionsParameters"] = msg.actions_actions_parameters;
  j["instantActions"] = msg.instant_actions;
  j["trajectoryKnotVector"] = msg.trajectory_knot_vector;
  j["trajectoryControlPoints"] = msg.trajectory_control_points;
  j["stateNodeStates"] = msg.state_node_states;
  j["stateEdgeStates"] = msg.state_edge_states;
  j["stateLoads"] = msg.state_loads;
  j["stateActionStates"] = msg.state_action_states;
  j["stateErrors"] = msg.state_errors;
  j["stateInformation"] = msg.state_information;
  j["errorErrorReferences"] = msg.error_error_references;
  j["informationInfoReferences"] = msg.information_info_references;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::MaxArrayLens object
///
/// \param j Reference to the JSON object containing serialized MaxArrayLens data
/// \param msg Reference to the MaxArrayLens message to populate
void from_json(const nlohmann::json& j, MaxArrayLens& msg)
{
  msg.order_nodes = j.at("orderNodes").get<uint32_t>();
  msg.order_edges = j.at("orderEdges").get<uint32_t>();
  msg.node_actions = j.at("nodeActions").get<uint32_t>();
  msg.edge_actions = j.at("edgeActions").get<uint32_t>();
  msg.actions_actions_parameters =
    j.at("actionsActionsParameters").get<uint32_t>();
  msg.instant_actions = j.at("instantActions").get<uint32_t>();
  msg.trajectory_knot_vector = j.at("trajectoryKnotVector").get<uint32_t>();
  msg.trajectory_control_points =
    j.at("trajectoryControlPoints").get<uint32_t>();
  msg.state_node_states = j.at("stateNodeStates").get<uint32_t>();
  msg.state_edge_states = j.at("stateEdgeStates").get<uint32_t>();
  msg.state_loads = j.at("stateLoads").get<uint32_t>();
  msg.state_action_states = j.at("stateActionStates").get<uint32_t>();
  msg.state_errors = j.at("stateErrors").get<uint32_t>();
  msg.state_information = j.at("stateInformation").get<uint32_t>();
  msg.error_error_references = j.at("errorErrorReferences").get<uint32_t>();
  msg.information_info_references =
    j.at("informationInfoReferences").get<uint32_t>();
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::MaxStringLens object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the MaxStringLens message object to serialize
void to_json(nlohmann::json& j, const MaxStringLens& msg)
{
  if (!msg.msg_len.empty())
  {
    j["msgLen"] = msg.msg_len;
  }

  if (!msg.topic_serial_len.empty())
  {
    j["actionTypeLen"] = msg.topic_serial_len;
  }

  if (!msg.topic_elem_len.empty())
  {
    j["actionScopesLen"] = msg.topic_elem_len;
  }

  if (!msg.id_len.empty())
  {
    j["keyLen"] = msg.id_len;
  }

  if (!msg.enum_len.empty())
  {
    j["valueDataTypeLen"] = msg.enum_len;
  }

  if (!msg.load_id_len.empty())
  {
    j["setNameLen"] = msg.load_id_len;
  }

  if (!msg.id_numerical_only.empty())
  {
    j["isOptionalLen"] = msg.id_numerical_only;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::MaxStringLens object
///
/// \param j Reference to the JSON object containing serialized MaxStringLens data
/// \param msg Reference to the MaxStringLens message to populate
void from_json(const nlohmann::json& j, MaxStringLens& msg)
{
  if (j.contains("msgLen"))
  {
    msg.msg_len.push_back(j.at("msgLen").get<uint32_t>());
  }

  if (j.contains("topicSerialLen"))
  {
    msg.topic_serial_len.push_back(j.at("topicSerialLen").get<uint32_t>());
  }

  if (j.contains("topicElemLen"))
  {
    msg.topic_elem_len.push_back(j.at("topicElemLen").get<uint32_t>());
  }

  if (j.contains("idLen"))
  {
    msg.id_len.push_back(j.at("idLen").get<uint32_t>());
  }

  if (j.contains("enumLen"))
  {
    msg.enum_len.push_back(j.at("enumLen").get<uint32_t>());
  }

  if (j.contains("loadIdLen"))
  {
    msg.load_id_len.push_back(j.at("loadIdLen").get<uint32_t>());
  }

  if (j.contains("idNumericalOnly"))
  {
    msg.id_numerical_only.push_back(j.at("idNumericalOnly").get<bool>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Network object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Network message object to serialize
void to_json(nlohmann::json& j, const Network& msg)
{
  if (!msg.dns_servers.empty())
  {
    j["dnsServers"] = msg.dns_servers;
  }

  if (!msg.ntp_servers.empty())
  {
    j["ntpServers"] = msg.ntp_servers;
  }

  if (!msg.local_ip_address.empty())
  {
    j["localIpAddress"] = msg.local_ip_address;
  }

  if (!msg.netmask.empty())
  {
    j["netmask"] = msg.netmask;
  }

  if (!msg.default_gateway.empty())
  {
    j["defaultGateway"] = msg.default_gateway;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Network object
///
/// \param j Reference to the JSON object containing serialized Network data
/// \param msg Reference to the Network message to populate
void from_json(const nlohmann::json& j, Network& msg)
{
  if (j.contains("dnsServers"))
  {
    msg.dns_servers = j.at("dnsServers").get<std::vector<std::string>>();
  }

  if (j.contains("ntpServers"))
  {
    msg.ntp_servers = j.at("ntpServers").get<std::vector<std::string>>();
  }

  if (j.contains("localIpAddress"))
  {
    msg.local_ip_address.push_back(j.at("localIpAddress").get<std::string>());
  }

  if (j.contains("netmask"))
  {
    msg.netmask.push_back(j.at("netmask").get<std::string>());
  }

  if (j.contains("defaultGateway"))
  {
    msg.default_gateway.push_back(j.at("defaultGateway").get<std::string>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::OptionalParameters object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the OptionalParameters message object to serialize
///
/// \throws std::runtime_error If failed to serialize support field
void to_json(nlohmann::json& j, const OptionalParameters& msg)
{
  j["parameter"] = msg.parameter;

  if (msg.support == "SUPPORTED" || msg.support == "REQUIRED")
  {
    j["support"] = msg.support;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected support in optional_parameters field");
  }

  if (!msg.description.empty())
  {
    j["description"] = msg.description;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::OptionalParameters object
///
/// \param j Reference to the JSON object containing serialized OptionalParameters data
/// \param msg Reference to the OptionalParameters message to populate
///
/// \throws std::runtime_error If failed to deserialize support field
void from_json(const nlohmann::json& j, OptionalParameters& msg)
{
  msg.parameter = j.at("parameter").get<std::string>();

  auto support = j.at("support").get<std::string>();
  if (support == "SUPPORTED" || support == "REQUIRED")
  {
    msg.support = support;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected support in optional_parameters field");
  }

  if (j.contains("description"))
  {
    msg.description.push_back(j.at("description").get<std::string>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::PhysicalParameters object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the PhysicalParameters message object to serialize
void to_json(nlohmann::json& j, const PhysicalParameters& msg)
{
  j["speedMin"] = msg.speed_min;
  j["speedMax"] = msg.speed_max;
  j["accelerationMax"] = msg.acceleration_max;
  j["decelerationMax"] = msg.deceleration_max;
  j["heightMin"] = msg.height_min;
  j["heightMax"] = msg.height_max;
  j["width"] = msg.width;
  j["length"] = msg.length;

  if (!msg.angular_speed_min.empty())
  {
    j["angularSpeedMin"] = msg.angular_speed_min;
  }

  if (!msg.angular_speed_max.empty())
  {
    j["angularSpeedMax"] = msg.angular_speed_max;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::PhysicalParameters object
///
/// \param j Reference to the JSON object containing serialized PhysicalParameters data
/// \param msg Reference to the PhysicalParameters message to populate
void from_json(const nlohmann::json& j, PhysicalParameters& msg)
{
  msg.speed_min = j.at("speedMin").get<double>();
  msg.speed_max = j.at("speedMax").get<double>();
  msg.acceleration_max = j.at("accelerationMax").get<double>();
  msg.deceleration_max = j.at("decelerationMax").get<double>();
  msg.height_min = j.at("heightMin").get<double>();
  msg.height_max = j.at("heightMax").get<double>();
  msg.width = j.at("width").get<double>();
  msg.length = j.at("length").get<double>();

  if (j.contains("angularSpeedMin"))
  {
    msg.angular_speed_min.push_back(j.at("angularSpeedMin").get<double>());
  }

  if (j.contains("angularSpeedMax"))
  {
    msg.angular_speed_max.push_back(j.at("angularSpeedMax").get<double>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::ProtocolFeatures object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the ProtocolFeatures message object to serialize
void to_json(nlohmann::json& j, const ProtocolFeatures& msg)
{
  j["optionalParameters"] = msg.optional_parameters;
  j["agvActions"] = msg.agv_actions;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::ProtocolFeatures object
///
/// \param j Reference to the JSON object containing serialized ProtocolFeatures data
/// \param msg Reference to the ProtocolFeatures message to populate
void from_json(const nlohmann::json& j, ProtocolFeatures& msg)
{
  msg.optional_parameters =
    j.at("optionalParameters").get<std::vector<OptionalParameters>>();
  msg.agv_actions = j.at("agvActions").get<std::vector<AgvAction>>();
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::ProtocolLimits object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the ProtocolLimits message object to serialize
void to_json(nlohmann::json& j, const ProtocolLimits& msg)
{
  j["maxStringLens"] = msg.max_string_lens;
  j["maxArrayLens"] = msg.max_array_lens;
  j["timing"] = msg.timing;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::ProtocolLimits object
///
/// \param j Reference to the JSON object containing serialized ProtocolLimits data
/// \param msg Reference to the ProtocolLimits message to populate
void from_json(const nlohmann::json& j, ProtocolLimits& msg)
{
  msg.max_string_lens = j.at("maxStringLens").get<MaxStringLens>();
  msg.max_array_lens = j.at("maxArrayLens").get<MaxArrayLens>();
  msg.timing = j.at("timing").get<Timing>();
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Timing object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Timing message object to serialize
void to_json(nlohmann::json& j, const Timing& msg)
{
  j["minOrderInterval"] = msg.min_order_interval;
  j["minStateInterval"] = msg.min_state_interval;

  if (!msg.default_state_interval.empty())
  {
    j["defaultStateInterval"] = msg.default_state_interval;
  }

  if (!msg.visualization_interval.empty())
  {
    j["visualizationInterval"] = msg.visualization_interval;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Timing object
///
/// \param j Reference to the JSON object containing serialized Timing data
/// \param msg Reference to the Timing message to populate
void from_json(const nlohmann::json& j, Timing& msg)
{
  msg.min_order_interval = j.at("minOrderInterval").get<double>();
  msg.min_state_interval = j.at("minStateInterval").get<double>();

  if (j.contains("defaultStateInterval"))
  {
    msg.default_state_interval.push_back(
      j.at("defaultStateInterval").get<double>());
  }

  if (j.contains("visualizationInterval"))
  {
    msg.visualization_interval.push_back(
      j.at("visualizationInterval").get<double>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::TypeSpecification object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the TypeSpecification message object to serialize
///
/// \throws std::runtime_error If failed to serialize agv_kinematic or agv_class field
void to_json(nlohmann::json& j, const TypeSpecification& msg)
{
  j["seriesName"] = msg.series_name;

  if (
    msg.agv_kinematic == "DIFF" || msg.agv_kinematic == "OMNI" ||
    msg.agv_kinematic == "THREEWHEEL")
  {
    j["agvKInematic"] = msg.agv_kinematic;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected agv_kinematic in type_specification "
      "field");
  }

  if (
    msg.agv_class == "FORKLIFT" || msg.agv_class == "CONVEYOR" ||
    msg.agv_class == "TUGGER" || msg.agv_class == "CARRIER")
  {
    j["agvClass"] = msg.agv_class;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected agv_class in type_specification field");
  }

  j["maxLoadMass"] = msg.max_load_mass;
  j["localizationTypes"] = msg.localization_types;
  j["navigationTypes"] = msg.navigation_types;

  if (!msg.series_description.empty())
  {
    j["seriesDescription"] = msg.series_description;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::TypeSpecification object
///
/// \param j Reference to the JSON object containing serialized TypeSpecification data
/// \param msg Reference to the TypeSpecification message to populate
///
/// \throws std::runtime_error If failed to deserialize agv_kinematic or agv_class field
void from_json(const nlohmann::json& j, TypeSpecification& msg)
{
  msg.series_name = j.at("seriesName").get<std::string>();

  auto agv_kinematic = j.at("agvKinematic").get<std::string>();
  if (
    agv_kinematic == "DIFF" || agv_kinematic == "OMNI" ||
    agv_kinematic == "THREEWHEEL")
  {
    msg.agv_kinematic = agv_kinematic;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected agv_kinematic in type_specification "
      "field");
  }

  auto agv_class = j.at("agvClass").get<std::string>();
  if (
    agv_class == "FORKLIFT" || agv_class == "CONVEYOR" ||
    agv_class == "TUGGER" || agv_class == "CARRIER")
  {
    msg.agv_class = agv_class;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected agv_class in type_specification field");
  }

  msg.max_load_mass = j.at("maxLoadMass").get<double>();
  msg.localization_types =
    j.at("localizationTypes").get<std::vector<std::string>>();
  msg.navigation_types =
    j.at("navigationTypes").get<std::vector<std::string>>();

  if (j.contains("seriesDescription"))
  {
    msg.series_description.push_back(
      j.at("seriesDescription").get<std::string>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::VehicleConfig object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the VehicleConfig message object to serialize
void to_json(nlohmann::json& j, const VehicleConfig& msg)
{
  if (!msg.versions.empty())
  {
    j["versions"] = msg.versions;
  }

  if (!msg.network.empty())
  {
    j["network"] = msg.network;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::VehicleConfig object
///
/// \param j Reference to the JSON object containing serialized VehicleConfig data
/// \param msg Reference to the VehicleConfig message to populate
void from_json(const nlohmann::json& j, VehicleConfig& msg)
{
  if (j.contains("versions"))
  {
    msg.versions = j.at("versions").get<std::vector<VersionInfo>>();
  }

  if (j.contains("network"))
  {
    msg.network.push_back(j.at("network").get<Network>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::VersionInfo object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the VersionInfo message object to serialize
void to_json(nlohmann::json& j, const VersionInfo& msg)
{
  j["key"] = msg.key;
  j["value"] = msg.value;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::VersionInfo object
///
/// \param j Reference to the JSON object containing serialized VersionInfo data
/// \param msg Reference to the VersionInfo message to populate
void from_json(const nlohmann::json& j, VersionInfo& msg)
{
  msg.key = j.at("key").get<std::string>();
  msg.value = j.at("value").get<std::string>();
}

// ============================================================================
/// \brief  Convert a vda5050_msgs::msg::Factsheet object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Factsheet message object to serialize
void to_json(nlohmann::json& j, const Factsheet& msg)
{
  to_json(j, msg.header);
  j["typeSpecification"] = msg.type_specification;
  j["physicalParameters"] = msg.physical_parameters;
  j["protocolLimits"] = msg.protocol_limits;
  j["protocolFeatures"] = msg.protocol_features;
  j["agvGeometry"] = msg.agv_geometry;
  j["loadSpecification"] = msg.load_specification;
  j["vehicleConfig"] = msg.vehicle_config;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Factsheet object
///
/// \param j Reference to the JSON object containing serialized Factsheet data
/// \param msg Reference to the Factsheet message to populate
void from_json(const nlohmann::json& j, Factsheet& msg)
{
  from_json(j, msg.header);
  msg.type_specification = j.at("typeSpecification").get<TypeSpecification>();
  msg.physical_parameters =
    j.at("physicalParameters").get<PhysicalParameters>();
  msg.protocol_limits = j.at("protocolLimits").get<ProtocolLimits>();
  msg.protocol_features = j.at("protocolFeatures").get<ProtocolFeatures>();
  msg.agv_geometry = j.at("agvGeometry").get<AgvGeometry>();
  msg.load_specification = j.at("loadSpecification").get<LoadSpecification>();
  msg.vehicle_config = j.at("vehicleConfig").get<VehicleConfig>();
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  /// VDA5050_MSGS__JSON_UTILS__FACTSHEET_HPP_
