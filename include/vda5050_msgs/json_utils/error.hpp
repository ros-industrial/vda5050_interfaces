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

#ifndef VDA5050_MSGS__JSON_UTILS__ERROR_HPP_
#define VDA5050_MSGS__JSON_UTILS__ERROR_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/error_reference.hpp"
#include "vda5050_msgs/msg/error.hpp"

namespace vda5050_msgs {

namespace msg {

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

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__ERROR_HPP_
