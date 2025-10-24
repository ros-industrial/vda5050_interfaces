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

#ifndef VDA5050_MSGS__JSON_UTILS__ACTION_PARAMETER_FACTSHEET_HPP_
#define VDA5050_MSGS__JSON_UTILS__ACTION_PARAMETER_FACTSHEET_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/action_parameter_factsheet.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::ActionParameterFactsheet object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize value_data_type
inline void to_json(nlohmann::json& j, const ActionParameterFactsheet& msg)
{
  j["key"] = msg.key;

  if (
    msg.value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_BOOL ||
    msg.value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_NUMBER ||
    msg.value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_INTEGER ||
    msg.value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_FLOAT ||
    msg.value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_OBJECT ||
    msg.value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_ARRAY)
  {
    j["valueDataType"] = msg.value_data_type;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected value_data_type");
  }

  if (!msg.description.empty())
  {
    j["description"] = msg.description.front();
  }

  if (!msg.is_optional.empty())
  {
    j["isOptional"] = msg.is_optional.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::ActionParameterFactsheet object
///
/// \param j Reference to the JSON object containing serialized ActionParameterFactsheet data
/// \param msg Reference to the ActionParameterFactsheet message to populate
///
/// \throws std::runtime_error If failed to deserialize value_data_type
inline void from_json(const nlohmann::json& j, ActionParameterFactsheet& msg)
{
  msg.key = j.at("key").get<std::string>();

  auto value_data_type = j.at("valueDataType").get<std::string>();
  if (
    value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_BOOL ||
    value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_NUMBER ||
    value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_INTEGER ||
    value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_FLOAT ||
    value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_OBJECT ||
    value_data_type == ActionParameterFactsheet::VALUE_DATA_TYPE_ARRAY)
  {
    msg.value_data_type = value_data_type;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected value_data_type");
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

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__ACTION_PARAMETER_FACTSHEET_HPP_
