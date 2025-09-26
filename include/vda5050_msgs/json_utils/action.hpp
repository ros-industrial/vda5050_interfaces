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

#ifndef VDA5050_MSGS__JSON_UTILS__ACTION_HPP_
#define VDA5050_MSGS__JSON_UTILS__ACTION_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/action_parameter.hpp"
#include "vda5050_msgs/msg/action.hpp"

namespace vda5050_msgs {

namespace msg {
//=============================================================================
/// \brief convert a vda5050_msgs::msg::Action object to a nlohmann::json object
///
/// \param j Reference to a JSON object to be populated
/// \param msg Reference to the messge object to serialize
///
/// \throws std::runtime_error If failed to serialize blockingType
void to_json(nlohmann::json& j, const Action& msg)
{
  j["actionType"] = msg.action_type;
  j["actionId"] = msg.action_id;

  if (
    msg.blocking_type == Action::NONE || msg.blocking_type == Action::SOFT ||
    msg.blocking_type == Action::HARD)
  {
    j["blockingType"] = msg.blocking_type;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected blockingType");
  }

  if (!msg.action_description.empty())
  {
    j["actionDescription"] = msg.action_description.front();
  }

  if (!msg.action_parameters.empty())
  {
    j["actionParameters"] = msg.action_parameters;
  }
}

//=============================================================================
/// \brief populate a vda5050_msgs::msg::Action object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized Action data
/// \param msg Reference to the Action message to populate
///
/// \throws std::runtime_error If failed to deserialize blockingType
void from_json(const nlohmann::json& j, Action& msg)
{
  msg.action_type = j.at("actionType").get<std::string>();
  msg.action_id = j.at("actionId").get<std::string>();

  auto blocking_type = j.at("blockingType").get<std::string>();
  if (
    blocking_type == Action::NONE || blocking_type == Action::SOFT ||
    blocking_type == Action::HARD)
  {
    msg.blocking_type = blocking_type;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected blockingType.");
  }

  if (j.contains("actionDescription"))
  {
    msg.action_description.push_back(
      j.at("actionDescription").get<std::string>());
  }

  if (j.contains("actionParameters"))
  {
    msg.action_parameters =
      j.at("actionParameters").get<std::vector<ActionParameter>>();
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  /// VDA5050_MSGS__JSON_UTILS__ACTION_HPP_