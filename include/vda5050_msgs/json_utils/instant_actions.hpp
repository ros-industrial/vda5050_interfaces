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

#ifndef VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_
#define VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/msg/instant_actions.hpp"

namespace vda5050_msgs {

namespace msg {

/// \brief Convert a vda5050_msgs::msg::ActionParameterValue object to a nlohmann::json object
///
/// \param j Reference to a JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize type
void to_json(nlohmann::json& j, const ActionParameterValue& msg)
{
  if (
    msg.type == ActionParameterValue::ARRAY ||
    msg.type == ActionParameterValue::BOOL ||
    msg.type == ActionParameterValue::NUMBER ||
    msg.type == ActionParameterValue::STRING ||
    msg.type == ActionParameterValue::OBJECT)
  {
    j["type"] = msg.type;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected type");
  }

  j["value"] = msg.value;
}

/// \brief Populate a vda5050_msgs::msg::ActionParameterValue object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized parameter value data
/// \param msg Reference to the ActionParameterValue message to populate
///
/// \throws std::runtime_error If failed to deserialize type
void from_json(const nlohmann::json& j, ActionParameterValue& msg)
{
  auto type = j.at("type").get<uint8_t>();
  if (
    type == ActionParameterValue::ARRAY || type == ActionParameterValue::BOOL ||
    type == ActionParameterValue::NUMBER ||
    type == ActionParameterValue::STRING ||
    type == ActionParameterValue::OBJECT)
  {
    msg.type = type;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected type.");
  }

  msg.value = j.at("value").get<std::string>();
}

/// \brief convert a vda5050_msgs::msg::ActionParameter object to a nlohmann::json object
///
/// \param j Reference to a JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const ActionParameter& msg)
{
  j["key"] = msg.key;
  j["value"] = msg.value;
}

/// \brief populate a vda5050_msgs::msg::ActionParameter object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized ActionParameter data
/// \param msg Reference to the ActionParameter message to populate
void from_json(const nlohmann::json& j, ActionParameter& msg)
{
  msg.key = j.at("key").get<std::string>();
  msg.value = j.at("value").get<ActionParameterValue>();
}

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

/// \brief convert a vda5050_msgs::msg::InstantAction object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const InstantActions& msg)
{
  to_json(j, msg.header);

  j["actions"] = msg.actions;
}

/// \brief populate a vda5050_msgs::msg::InstantAction object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized InstantAction data
/// \param msg Reference to the InstantAction object to be populated
void from_json(const nlohmann::json& j, InstantActions& msg)
{
  from_json(j, msg.header);

  msg.actions = j.at("actions").get<std::vector<Action>>();
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_
