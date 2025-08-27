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

#ifndef VDA5050_MSGS__JSON_UTILS__STATE_HPP
#define VDA5050_MSGS__JSON_UTILS__STATE_HPP

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/msg/state.hpp"

namespace vda5050_msgs {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::AGVPosition object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const AGVPosition& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["theta"] = msg.theta;
  j["mapId"] = msg.map_id;
  j["positionInitialized"] = msg.position_initialized;

  if (!msg.map_description.empty())
  {
    j["mapDescription"] = msg.map_description.front();
  }

  if (!msg.localization_score.empty())
  {
    j["localizationScore"] = msg.localization_score.front();
  }

  if (!msg.deviation_range.empty())
  {
    j["deviationRange"] = msg.deviation_range.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::AGVPosition object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, AGVPosition& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.theta = j.at("theta").get<double>();
  msg.map_id = j.at("mapId").get<std::string>();
  msg.position_initialized = j.at("positionInitialized").get<bool>();

  if (j.contains("mapDescription"))
  {
    msg.map_description.push_back(j.at("mapDescription").get<std::string>());
  }

  if (j.contains("localizationScore"))
  {
    msg.localization_score.push_back(j.at("localizationScore").get<double>());
  }

  if (j.contains("deviationRange"))
  {
    msg.deviation_range.push_back(j.at("deviationRange").get<double>());
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Velocity object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const Velocity& msg)
{
  if (!msg.vx.empty())
  {
    j["vx"] = msg.vx.front();
  }

  if (!msg.vy.empty())
  {
    j["vy"] = msg.vy.front();
  }

  if (!msg.omega.empty())
  {
    j["omega"] = msg.omega.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Velocity object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, Velocity& msg)
{
  if (j.contains("vx"))
  {
    msg.vx.push_back(j.at("vx").get<double>());
  }

  if (j.contains("vy"))
  {
    msg.vy.push_back(j.at("vy").get<double>());
  }

  if (j.contains("omega"))
  {
    msg.omega.push_back(j.at("omega").get<double>());
  }
}

//=============================================================================
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

//=============================================================================
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

//=============================================================================
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

//=============================================================================
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

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Load object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const Load& msg)
{
  if (!msg.load_id.empty())
  {
    j["loadId"] = msg.load_id.front();
  }

  if (!msg.load_type.empty())
  {
    j["loadType"] = msg.load_type.front();
  }

  if (!msg.load_position.empty())
  {
    j["loadPosition"] = msg.load_position.front();
  }

  if (!msg.bounding_box_reference.empty())
  {
    j["boundingBoxReference"] = msg.bounding_box_reference.front();
  }

  if (!msg.load_dimensions.empty())
  {
    j["LoadDimensions"] = msg.load_dimensions.front();
  }

  if (!msg.weight.empty())
  {
    j["weight"] = msg.weight.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Load object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, Load& msg)
{
  if (j.contains("loadId"))
  {
    msg.load_id.push_back(j.at("loadId").get<std::string>());
  }

  if (j.contains("loadType"))
  {
    msg.load_type.push_back(j.at("loadType").get<std::string>());
  }

  if (j.contains("loadPosition"))
  {
    msg.load_position.push_back(j.at("loadPosition").get<std::string>());
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

  if (j.contains("weight"))
  {
    msg.weight.push_back(j.at("weight").get<double>());
  }
}

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

  if (!msg.theta.empty())
  {
    j["theta"] = msg.theta.front();
  }

  if (!msg.allowed_deviation_x_y.empty())
  {
    j["allowedDeviationXY"] = msg.allowed_deviation_x_y.front();
  }

  if (!msg.allowed_deviation_theta.empty())
  {
    j["allowedDeviationTheta"] = msg.allowed_deviation_theta.front();
  }

  if (!msg.map_description.empty())
  {
    j["mapDescription"] = msg.map_description.front();
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

  if (j.contains("theta"))
  {
    msg.theta.push_back(j.at("theta").get<double>());
  }

  if (j.contains("allowedDeviationXY"))
  {
    msg.allowed_deviation_x_y.push_back(
      j.at("allowedDeviationXY").get<double>());
  }

  if (j.contains("allowedDeviationTheta"))
  {
    msg.allowed_deviation_theta.push_back(
      j.at("allowedDeviationTheta").get<double>());
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

  if (!msg.node_description.empty())
  {
    j["nodeDescription"] = msg.node_description;
  }

  if (!msg.node_position.empty())
  {
    j["nodePosition"] = msg.node_position.front();
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
    msg.node_description.push_back(j.at("nodeDescription").get<std::string>());
  }

  if (j.contains("nodePosition"))
  {
    msg.node_position.push_back(j.at("nodePosition").get<NodePosition>());
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
/// \brief Convert a vda5050_msgs::msg::Trajectory object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const Trajectory& msg)
{
  j["knotVector"] = msg.knot_vector;
  j["controlPoints"] = msg.control_points;

  if (!msg.degree.empty())
  {
    j["degree"] = msg.degree.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Trajectory object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, Trajectory& msg)
{
  msg.knot_vector = j.at("knotVector").get<std::vector<double>>();
  msg.control_points = j.at("controlPoints").get<std::vector<ControlPoint>>();

  if (j.contains("degree"))
  {
    msg.degree.push_back(j.at("degree").get<double>());
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::EdgeState object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const EdgeState& msg)
{
  j["edgeId"] = msg.edge_id;
  j["sequenceId"] = msg.sequence_id;
  j["released"] = msg.sequence_id;

  if (!msg.edge_description.empty())
  {
    j["edgeDescription"] = msg.edge_description.front();
  }

  if (!msg.trajectory.empty())
  {
    j["trajectory"] = msg.trajectory.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::EdgeState object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, EdgeState& msg)
{
  msg.edge_id = j.at("edgeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<int32_t>();
  msg.released = j.at("released").get<bool>();

  if (j.contains("edgeDescription"))
  {
    msg.edge_description.push_back(j.at("edgeDescription").get<std::string>());
  }

  if (j.contains("trajectory"))
  {
    msg.trajectory.push_back(j.at("trajectory").get<Trajectory>());
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::ActionState object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize actionStatus
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

  if (!msg.action_type.empty())
  {
    j["actionType"] = msg.action_type.front();
  }

  if (!msg.action_description.empty())
  {
    j["actionDescription"] = msg.action_description.front();
  }

  if (!msg.result_declaration.empty())
  {
    j["resultDeclaration"] = msg.result_declaration.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::ActionState object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize actionStatus
void from_json(const nlohmann::json& j, ActionState& msg)
{
  msg.action_id = j.at("actionId").get<std::string>();

  auto action_status = j.at("actionStatus").get<std::string>();
  if (
    action_status == ActionState::ACTION_STATUS_WAITING ||
    action_status == ActionState::ACTION_STATUS_INITIALIZING ||
    action_status == ActionState::ACTION_STATUS_RUNNING ||
    action_status == ActionState::ACTION_STATUS_PAUSED ||
    action_status == ActionState::ACTION_STATUS_FINISHED ||
    action_status == ActionState::ACTION_STATUS_FAILED)
  {
    msg.action_status = action_status;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected actionStatus");
  }

  if (j.contains("actionType"))
  {
    msg.action_type.push_back(j.at("actionType").get<std::string>());
  }

  if (j.contains("actionDescription"))
  {
    msg.action_description.push_back(
      j.at("actionDescription").get<std::string>());
  }

  if (j.contains("resultDeclaration"))
  {
    msg.result_declaration.push_back(
      j.at("resultDeclaration").get<std::string>());
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::BatteryState object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const BatteryState& msg)
{
  j["batteryCharge"] = msg.battery_charge;
  j["charging"] = msg.charging;

  if (!msg.battery_voltage.empty())
  {
    j["batteryVoltage"] = msg.battery_voltage.front();
  }

  if (!msg.battery_health.empty())
  {
    j["batteryHealth"] = msg.battery_health.front();
  }

  if (!msg.reach.empty())
  {
    j["reach"] = msg.reach.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::BatteryState object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, BatteryState& msg)
{
  msg.battery_charge = j.at("batteryCharge").get<double>();
  msg.charging = j.at("charging").get<bool>();

  if (j.contains("batteryVoltage"))
  {
    msg.battery_voltage.push_back(j.at("batteryVoltage").get<double>());
  }

  if (j.contains("batteryHealth"))
  {
    msg.battery_health.push_back(j.at("batteryHealth").get<double>());
  }

  if (j.contains("reach"))
  {
    msg.reach.push_back(j.at("reach").get<uint32_t>());
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::ErrorReference object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const ErrorReference& msg)
{
  j["referenceKey"] = msg.reference_key;
  j["referenceValue"] = msg.reference_value;
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::ErrorReference object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, ErrorReference& msg)
{
  msg.reference_key = j.at("referenceKey").get<std::string>();
  msg.reference_value = j.at("referenceValue").get<std::string>();
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Error object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize errorLevel
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

  if (!msg.error_description.empty())
  {
    j["errorDescription"] = msg.error_description.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Error object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize errorLevel
void from_json(const nlohmann::json& j, Error& msg)
{
  msg.error_type = j.at("errorType").get<std::string>();

  auto error_level = j.at("errorLevel").get<std::string>();
  if (
    error_level == Error::ERROR_LEVEL_WARNING ||
    error_level == Error::ERROR_LEVEL_FATAL)
  {
    msg.error_level = error_level;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected errorLevel");
  }

  if (j.contains("errorReferences"))
  {
    msg.error_references =
      j.at("errorReferences").get<std::vector<ErrorReference>>();
  }

  if (j.contains("errorDescription"))
  {
    msg.error_description.push_back(
      j.at("errorDescription").get<std::string>());
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::InfoReference object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const InfoReference& msg)
{
  j["referenceKey"] = msg.reference_key;
  j["referenceValue"] = msg.reference_value;
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::InfoReference object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, InfoReference& msg)
{
  msg.reference_key = j.at("referenceKey").get<std::string>();
  msg.reference_value = j.at("referenceValue").get<std::string>();
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Information object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize infoLevel
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

  if (!msg.info_description.empty())
  {
    j["infoDescription"] = msg.info_description.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Information object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize infoLevel
void from_json(const nlohmann::json& j, Information& msg)
{
  msg.info_type = j.at("infoType").get<std::string>();

  auto info_level = j.at("infoLevel").get<std::string>();
  if (
    info_level == Information::INFO_LEVEL_INFO ||
    info_level == Information::INFO_LEVEL_DEBUG)
  {
    msg.info_level = info_level;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected infoLevel");
  }

  if (j.contains("infoReferences"))
  {
    msg.info_references =
      j.at("infoReferences").get<std::vector<InfoReference>>();
  }

  if (j.contains("infoDescription"))
  {
    msg.info_description.push_back(j.at("infoDescription").get<std::string>());
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::SafetyState object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize eStop
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
/// \brief Populate a vda5050_msgs::msg::SafetyState object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize eStop
void from_json(const nlohmann::json& j, SafetyState& msg)
{
  auto e_stop = j.at("eStop").get<std::string>();
  if (
    e_stop == SafetyState::E_STOP_AUTOACK ||
    e_stop == SafetyState::E_STOP_MANUAL ||
    e_stop == SafetyState::E_STOP_REMOTE || e_stop == SafetyState::E_STOP_NONE)
  {
    msg.e_stop = e_stop;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected state for eStop");
  }

  msg.field_violation = j.at("fieldViolation").get<bool>();
}

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
  j["errors"] = msg.errors;

  if (!msg.zone_set_id.empty())
  {
    j["zoneSetId"] = msg.zone_set_id.front();
  }

  if (!msg.paused.empty())
  {
    j["paused"] = msg.paused.front();
  }

  if (!msg.new_base_request.empty())
  {
    j["newBaseRequest"] = msg.new_base_request.front();
  }

  if (!msg.distance_since_last_node.empty())
  {
    j["distanceSinceLastNode"] = msg.distance_since_last_node.front();
  }

  if (!msg.agv_position.empty())
  {
    j["agvPosition"] = msg.agv_position.front();
  }

  if (!msg.velocity.empty())
  {
    j["velocity"] = msg.velocity.front();
  }

  if (!msg.loads.empty())
  {
    j["loads"] = msg.loads;
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
  msg.errors = j.at("errors").get<std::vector<Error>>();

  if (j.contains("zoneSetId"))
  {
    msg.zone_set_id.push_back(j.at("zoneSetId").get<std::string>());
  }

  if (j.contains("paused"))
  {
    msg.paused.push_back(j.at("paused").get<bool>());
  }

  if (j.contains("newBaseRequest"))
  {
    msg.new_base_request.push_back(j.at("newBaseRequest").get<bool>());
  }

  if (j.contains("distanceSinceLastNode"))
  {
    msg.distance_since_last_node.push_back(
      j.at("distanceSinceLastNode").get<double>());
  }

  if (j.contains("agvPosition"))
  {
    msg.agv_position.push_back(j.at("agvPosition").get<AGVPosition>());
  }

  if (j.contains("velocity"))
  {
    msg.velocity.push_back(j.at("velocity").get<Velocity>());
  }

  if (j.contains("loads"))
  {
    msg.loads = j.at("loads").get<std::vector<Load>>();
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__STATE_HPP
