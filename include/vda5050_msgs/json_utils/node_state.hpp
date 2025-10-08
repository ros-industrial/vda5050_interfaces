/*
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

#ifndef VDA5050_MSGS__JSON_UTILS__NODE_STATE_HPP_
#define VDA5050_MSGS__JSON_UTILS__NODE_STATE_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/node_position.hpp"
#include "vda5050_msgs/msg/node_state.hpp"

#include "vda5050_msgs/json_utils/node_position.hpp"

namespace vda5050_msgs {

namespace msg {

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
    j["nodeDescription"] = msg.node_description.front();
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
  msg.sequence_id = j.at("sequenceId").get<uint32_t>();
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

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__NODE_STATE_HPP_
