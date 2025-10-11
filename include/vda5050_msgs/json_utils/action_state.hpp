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

#ifndef VDA5050_MSGS__JSON_UTILS__ACTION_STATE_HPP_
#define VDA5050_MSGS__JSON_UTILS__ACTION_STATE_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/action_state.hpp"

namespace vda5050_msgs {

namespace msg {

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

  if (!msg.result_description.empty())
  {
    j["resultDescription"] = msg.result_description.front();
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

  if (j.contains("resultDescription"))
  {
    msg.result_description.push_back(
      j.at("resultDescription").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__ACTION_STATE_HPP_
