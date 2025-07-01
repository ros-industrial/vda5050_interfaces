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

#ifndef VDA5050_MSGS__JSON_UTILS__STATE_HPP_
#define VDA5050_MSGS__JSON_UTILS__STATE_HPP_

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/msg/state.hpp"

namespace vda5050_msgs {
namespace msg {

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::NodePosition object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const NodePosition& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["mapId"] = msg.map_id;

  if (msg.theta.value_initialized)
  {
    j["theta"] = msg.theta.value;
  }

  if (msg.allowed_deviation_xy.value_initialized)
  {
    j["allowedDeviationXY"] = msg.allowed_deviation_xy.value;
  }

  if (msg.allowed_deviation_theta.value_initialized)
  {
    j["allowedDeviationTheta"] = msg.allowed_deviation_theta.value;
  }

  if (msg.map_description.value_initialized)
  {
    j["mapDescription"] = msg.map_description.value;
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::NodePosition object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, NodePosition& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.map_id = j.at("mapId").get<std::string>();
  msg.value_initialized = true;

  if (j.contains("theta"))
  {
    msg.theta.value = j.at("theta").get<double>();
    msg.theta.value_initialized = true;
  }

  if (j.contains("allowedDeviationXY"))
  {
    msg.allowed_deviation_xy.value = j.at("allowedDeviationXY").get<double>();
    msg.allowed_deviation_xy.value_initialized = true;
  }

  if (j.contains("allowedDeviationTheta"))
  {
    msg.allowed_deviation_theta.value =
      j.at("allowedDeviationTheta").get<double>();
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::NodeState object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const NodeState& msg)
{
  j["nodeId"] = msg.node_id;
  j["sequenceId"] = msg.sequence_id;
  j["released"] = msg.released;

  if (msg.node_description.value_initialized)
  {
    j["nodeDescription"] = msg.node_description.value;
  }

  if (msg.node_position.value_initialized)
  {
    j["nodePosition"] = msg.node_position;
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::NodeState object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, NodeState& msg)
{
  msg.node_id = j.at("nodeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<int32_t>();
  msg.released = j.at("released").get<bool>();

  if (j.contains("nodeDescription"))
  {
    msg.node_description.value = j.at("nodeDescription").get<std::string>();
    msg.node_description.value_initialized = true;
  }

  if (j.contains("nodePosition"))
  {
    msg.node_position = j.at("nodePosition").get<NodePosition>();
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::ControlPoint object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const ControlPoint& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["weight"] = msg.weight;
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::ControlPoint object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, ControlPoint& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();

  if (j.contains("weight"))
  {
    msg.weight = j.at("weight").get<double>();
  }
}

//=============================================================================
void to_json(nlohmann::json& j, const Trajectory& msg)
{
  j["knotVector"] = msg.knot_vector;
  j["controlPoints"] = msg.control_points;

  if (msg.degree.value_initialized)
  {
    j["degree"] = msg.degree.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, Trajectory& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const EdgeState& msg)
{
  j["edgeId"] = msg.edge_id;
  j["sequenceId"] = msg.sequence_id;
  j["released"] = msg.released;

  if (msg.edge_description.value_initialized)
  {
    j["edgeDescription"] = msg.edge_description.value;
  }

  if (msg.trajectory.value_initialized)
  {
    j["trajectory"] = msg.trajectory;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, EdgeState& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const AGVPosition& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["theta"] = msg.theta;
  j["mapId"] = msg.map_id;
  j["positionInitialized"] = msg.position_initialized;

  if (msg.map_description.value_initialized)
  {
    j["mapDescription"] = msg.map_description.value;
  }

  if (msg.localization_score.value_initialized)
  {
    j["localizationScore"] = msg.localization_score.value;
  }

  if (msg.deviation_range.value_initialized)
  {
    j["deviationRange"] = msg.deviation_range.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, AGVPosition& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const Velocity& msg)
{
  if (msg.vx.value_initialized)
  {
    j["vx"] = msg.vx.value;
  }

  if (msg.vy.value_initialized)
  {
    j["vy"] = msg.vy.value;
  }

  if (msg.omega.value_initialized)
  {
    j["omega"] = msg.omega.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, Velocity& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const BoundingBoxReference& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["z"] = msg.z;

  if (msg.theta.value_initialized)
  {
    j["theta"] = msg.theta.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, BoundingBoxReference& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const LoadDimensions& msg)
{
  j["length"] = msg.length;
  j["width"] = msg.width;

  if (msg.height.value_initialized)
  {
    j["height"] = msg.height.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, LoadDimensions& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const Load& msg)
{
  if (msg.load_id.value_initialized)
  {
    j["loadId"] = msg.load_id.value;
  }

  if (msg.load_type.value_initialized)
  {
    j["loadType"] = msg.load_type.value;
  }

  if (msg.load_position.value_initialized)
  {
    j["loadPosition"] = msg.load_position.value;
  }

  if (msg.bounding_box_reference.value_initialized)
  {
    j["boundingBoxReference"] = msg.bounding_box_reference;
  }

  if (msg.load_dimensions.value_initialized)
  {
    j["loadDimensions"] = msg.load_dimensions;
  }

  if (msg.weight.value_initialized)
  {
    j["weight"] = msg.weight.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, Load& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const ActionState& msg)
{
  j["actionId"] = msg.action_id;

  if (
    msg.action_status == ActionState::ACTION_STATUS_WAITING ||
    msg.action_status == ActionState::ACTION_STATUS_INITIALIZING ||
    msg.action_status == ActionState::ACTION_STATUS_RUNNING ||
    msg.action_status == ActionState::ACTION_STATUS_PAUSED ||
    msg.action_status == ActionState::ACTION_STATUS_FINISHED ||
    msg.action_status == ActionState::ACTION_STATUS_FAILED)
  {
    j["actionStatus"] = msg.action_status;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected actionStatus");
  }

  if (msg.action_type.value_initialized)
  {
    j["actionType"] = msg.action_type.value;
  }

  if (msg.action_description.value_initialized)
  {
    j["actionDecription"] = msg.action_description.value;
  }

  if (msg.result_declaration.value_initialized)
  {
    j["resultDeclaration"] = msg.result_declaration.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, ActionState& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const BatteryState& msg)
{
  j["batteryCharge"] = msg.battery_charge;
  j["charging"] = msg.charging;

  if (msg.battery_voltage.value_initialized)
  {
    j["batteryVoltage"] = msg.battery_voltage.value;
  }

  if (msg.battery_health.value_initialized)
  {
    j["batteryHealth"] = msg.battery_health.value;
  }

  if (msg.reach.value_initialized)
  {
    j["reach"] = msg.reach.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, BatteryState& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const ErrorReference& msg)
{
  j["referenceKey"] = msg.reference_key;
  j["referenceValue"] = msg.reference_value;
}

//=============================================================================
void from_json(const nlohmann::json& j, ErrorReference& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const Error& msg)
{
  j["errorType"] = msg.error_type;

  if (
    msg.error_level == Error::ERROR_LEVEL_WARNING ||
    msg.error_level == Error::ERROR_LEVEL_FATAL)
  {
    j["errorLevel"] = msg.error_level;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected errorLevel");
  }

  if (!msg.error_references.empty())
  {
    j["errorReferences"] = msg.error_references;
  }

  if (msg.error_description.value_initialized)
  {
    j["errorDescription"] = msg.error_description.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, Error& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const InfoReference& msg)
{
  j["referenceKey"] = msg.reference_key;
  j["referenceValue"] = msg.reference_value;
}

//=============================================================================
void from_json(const nlohmann::json& j, InfoReference& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const Information& msg)
{
  j["infoType"] = msg.info_type;

  if (
    msg.info_level == Information::INFO_LEVEL_INFO ||
    msg.info_level == Information::INFO_LEVEL_DEBUG)
  {
    j["infoLevel"] = msg.info_level;
  }
  else
  {
    throw std::runtime_error("Serialiation error: Unexpected infoLevel");
  }

  if (!msg.info_references.empty())
  {
    j["infoReferences"] = msg.info_references;
  }

  if (msg.info_description.value_initialized)
  {
    j["infoDescription"] = msg.info_description.value;
  }
}

//=============================================================================
void from_json(const nlohmann::json& j, Information& msg) {}

//=============================================================================
void to_json(nlohmann::json& j, const SafetyState& msg)
{
  if (
    msg.e_stop == SafetyState::E_STOP_AUTOACK ||
    msg.e_stop == SafetyState::E_STOP_MANUAL ||
    msg.e_stop == SafetyState::E_STOP_REMOTE ||
    msg.e_stop == SafetyState::E_STOP_NONE)
  {
    j["eStop"] = msg.e_stop;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected state for eStop");
  }

  j["fieldViolation"] = msg.field_violation;
}

//=============================================================================
void from_json(const nlohmann::json& j, SafetyState& msg) {}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::State object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize operatingMode
void to_json(nlohmann::json& j, const State& msg)
{
  to_json(j, msg.header);

  j["orderId"] = msg.order_id;
  j["orderUpdateId"] = msg.order_update_id;
  j["lastNodeId"] = msg.last_node_id;
  j["lastNodeSequenceId"] = msg.last_node_sequence_id;
  j["driving"] = msg.driving;

  if (
    msg.operating_mode == State::OPERATING_MODE_AUTOMATIC ||
    msg.operating_mode == State::OPERATING_MODE_SEMIAUTOMATIC ||
    msg.operating_mode == State::OPERATING_MODE_MANUAL ||
    msg.operating_mode == State::OPERATING_MODE_SERVICE ||
    msg.operating_mode == State::OPERATING_MODE_TEACHIN)
  {
    j["operatingMode"] = msg.operating_mode;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected operatingMode");
  }

  j["nodeStates"] = msg.node_states;
  j["edgeStates"] = msg.edge_states;
  j["actionStates"] = msg.action_states;
  j["batteryState"] = msg.battery_state;
  j["errors"] = msg.errors;
  j["safetyState"] = msg.safety_state;

  if (msg.zone_set_id.value_initialized)
  {
    j["zoneSetId"] = msg.zone_set_id.value;
  }

  if (msg.paused.value_initialized)
  {
    j["paused"] = msg.paused.value;
  }

  if (msg.new_base_request.value_initialized)
  {
    j["newBaseRequest"] = msg.new_base_request.value;
  }

  if (msg.distance_since_last_node.value_initialized)
  {
    j["distanceSinceLastNode"] = msg.distance_since_last_node.value;
  }

  if (msg.agv_position.value_initialized)
  {
    j["agvPosition"] = msg.agv_position;
  }

  if (msg.velocity.value_initialized)
  {
    j["velocity"] = msg.velocity;
  }

  if (!msg.loads.empty())
  {
    j["loads"] = msg.loads;
  }

  if (!msg.information.empty())
  {
    j["information"] = msg.information;
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::State object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize operatingMode
void from_json(const nlohmann::json& j, State& msg)
{
  from_json(j, msg.header);

  msg.order_id = j.at("orderId").get<std::string>();
  msg.order_update_id = j.at("orderUpdateId").get<uint32_t>();
  msg.last_node_id = j.at("lastNodeId").get<std::string>();
  msg.last_node_sequence_id = j.at("lastNodeSequenceId").get<uint32_t>();
  msg.driving = j.at("driving").get<bool>();

  auto operating_mode = j.at("operatingMode").get<std::string>();
  if (
    operating_mode == State::OPERATING_MODE_AUTOMATIC ||
    operating_mode == State::OPERATING_MODE_SEMIAUTOMATIC ||
    operating_mode == State::OPERATING_MODE_MANUAL ||
    operating_mode == State::OPERATING_MODE_SERVICE ||
    operating_mode == State::OPERATING_MODE_TEACHIN)
  {
    msg.operating_mode = operating_mode;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected operatingMode");
  }

  msg.node_states = j.at("nodeStates").get<std::vector<NodeState>>();
  msg.edge_states = j.at("edgeStates").get<std::vector<EdgeState>>();
  msg.action_states = j.at("actionStates").get<std::vector<ActionState>>();
  msg.battery_state = j.at("batteryState").get<BatteryState>();
  msg.errors = j.at("errors").get<std::vector<Error>>();
  msg.safety_state = j.at("safetyState").get<SafetyState>();

  if (j.contains("zoneSetId"))
  {
    msg.zone_set_id.value = j.at("zoneSetId").get<std::string>();
    msg.zone_set_id.value_initialized = true;
  }

  if (j.contains("paused"))
  {
    msg.paused.value = j.at("paused").get<bool>();
    msg.paused.value_initialized = true;
  }

  if (j.contains("newBaseRequest"))
  {
    msg.new_base_request.value = j.at("newBaseRequest").get<bool>();
    msg.new_base_request.value_initialized = true;
  }

  if (j.contains("distanceSinceLastNode"))
  {
    msg.distance_since_last_node.value =
      j.at("distanceSinceLastNode").get<double>();
    msg.distance_since_last_node.value_initialized = true;
  }

  if (j.contains("agvPosition"))
  {
    msg.agv_position = j.at("agvPosition").get<AGVPosition>();
  }

  if (j.contains("velocity"))
  {
    msg.velocity = j.at("velocity").get<Velocity>();
  }

  if (j.contains("loads"))
  {
    msg.loads = j.at("loads").get<std::vector<Load>>();
  }

  if (j.contains("information"))
  {
    msg.information = j.at("information").get<std::vector<Information>>();
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__STATE_HPP_
