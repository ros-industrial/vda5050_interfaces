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

#ifndef VDA5050_INTERFACES__JSON_UTILS__NODE_HPP_
#define VDA5050_INTERFACES__JSON_UTILS__NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_interfaces/json_utils/action.hpp"
#include "vda5050_interfaces/json_utils/node_position.hpp"
#include "vda5050_interfaces/msg/node.hpp"

namespace vda5050_interfaces {

namespace msg {
//=============================================================================
/// \brief Convert a vda5050_interfaces::msg::Node object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
inline void to_json(nlohmann::json& j, const Node& msg)
{
  j["nodeId"] = msg.node_id;
  j["sequenceId"] = msg.sequence_id;
  j["released"] = msg.released;
  j["actions"] = msg.actions;

  if (!msg.node_position.empty())
  {
    j["nodePosition"] = msg.node_position.front();
  }

  if (!msg.node_description.empty())
  {
    j["nodeDescription"] = msg.node_description.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_interfaces::msg::Node object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
inline void from_json(const nlohmann::json& j, Node& msg)
{
  msg.node_id = j.at("nodeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<uint32_t>();
  msg.released = j.at("released").get<bool>();
  msg.actions = j.at("actions").get<std::vector<Action>>();

  if (j.contains("nodePosition"))
  {
    msg.node_position.push_back(j.at("nodePosition").get<NodePosition>());
  }

  if (j.contains("nodeDescription"))
  {
    msg.node_description.push_back(j.at("nodeDescription").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_interfaces

#endif  // VDA5050_INTERFACES__JSON_UTILS__NODE_HPP_
