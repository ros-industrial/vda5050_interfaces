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

#ifndef VDA5050_MSGS__JSON_UTILS__ACTION_PARAMETER_VALUE_HPP_
#define VDA5050_MSGS__JSON_UTILS__ACTION_PARAMETER_VALUE_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/action_parameter_value.hpp"

namespace vda5050_msgs {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::ActionParameterValue object to a nlohmann::json object
///
/// \param j Reference to a JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize type
void to_json(nlohmann::json& j, const ActionParameterValue& msg)
{
  if (
    msg.type == ActionParameterValue::TYPE_ARRAY ||
    msg.type == ActionParameterValue::TYPE_BOOL ||
    msg.type == ActionParameterValue::TYPE_NUMBER ||
    msg.type == ActionParameterValue::TYPE_STRING ||
    msg.type == ActionParameterValue::TYPE_OBJECT)
  {
    j["type"] = msg.type;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected type");
  }

  j["value"] = msg.value;
}

//=============================================================================
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
    type == ActionParameterValue::TYPE_ARRAY ||
    type == ActionParameterValue::TYPE_BOOL ||
    type == ActionParameterValue::TYPE_NUMBER ||
    type == ActionParameterValue::TYPE_STRING ||
    type == ActionParameterValue::TYPE_OBJECT)
  {
    msg.type = type;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected type.");
  }

  msg.value = j.at("value").get<std::string>();
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  /// VDA5050_MSGS__JSON_UTILS__ACTION_PARAMETER_VALUE_HPP_