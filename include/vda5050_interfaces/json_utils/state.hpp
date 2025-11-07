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

#ifndef VDA5050_INTERFACES__JSON_UTILS__STATE_HPP_
#define VDA5050_INTERFACES__JSON_UTILS__STATE_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_interfaces/msg/header.hpp"
#include "vda5050_interfaces/msg/state.hpp"

#include "vda5050_interfaces/json_utils/action_state.hpp"
#include "vda5050_interfaces/json_utils/agv_position.hpp"
#include "vda5050_interfaces/json_utils/battery_state.hpp"
#include "vda5050_interfaces/json_utils/bounding_box_reference.hpp"
#include "vda5050_interfaces/json_utils/control_point.hpp"
#include "vda5050_interfaces/json_utils/edge_state.hpp"
#include "vda5050_interfaces/json_utils/error.hpp"
#include "vda5050_interfaces/json_utils/error_reference.hpp"
#include "vda5050_interfaces/json_utils/header.hpp"
#include "vda5050_interfaces/json_utils/info.hpp"
#include "vda5050_interfaces/json_utils/info_reference.hpp"
#include "vda5050_interfaces/json_utils/load.hpp"
#include "vda5050_interfaces/json_utils/load_dimensions.hpp"
#include "vda5050_interfaces/json_utils/node_position.hpp"
#include "vda5050_interfaces/json_utils/node_state.hpp"
#include "vda5050_interfaces/json_utils/safety_state.hpp"
#include "vda5050_interfaces/json_utils/trajectory.hpp"
#include "vda5050_interfaces/json_utils/velocity.hpp"

namespace vda5050_interfaces {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_interfaces::msg::State object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize operatingMode
inline void to_json(nlohmann::json& j, const State& msg)
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
  j["batteryState"] = msg.battery_state;
  j["safetyState"] = msg.safety_state;

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

  if (!msg.information.empty())
  {
    j["information"] = msg.information;
  }
}

//=============================================================================
/// \brief Populate a vda5050_interfaces::msg::State object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize operatingMode
inline void from_json(const nlohmann::json& j, State& msg)
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
  msg.battery_state = j.at("batteryState").get<BatteryState>();
  msg.safety_state = j.at("safetyState").get<SafetyState>();

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

  if (j.contains("information"))
  {
    msg.information = j.at("information").get<std::vector<Info>>();
  }
}

}  // namespace msg
}  // namespace vda5050_interfaces

#endif  // VDA5050_INTERFACES__JSON_UTILS__STATE_HPP_
