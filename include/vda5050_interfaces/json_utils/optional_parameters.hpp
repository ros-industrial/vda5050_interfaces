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

#ifndef VDA5050_INTERFACES__JSON_UTILS__OPTIONAL_PARAMETERS_HPP_
#define VDA5050_INTERFACES__JSON_UTILS__OPTIONAL_PARAMETERS_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_interfaces/msg/optional_parameters.hpp"

namespace vda5050_interfaces {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_interfaces::msg::OptionalParameters object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the OptionalParameters message object to serialize
///
/// \throws std::runtime_error If failed to serialize support field
inline void to_json(nlohmann::json& j, const OptionalParameters& msg)
{
  j["parameter"] = msg.parameter;

  if (msg.support == "SUPPORTED" || msg.support == "REQUIRED")
  {
    j["support"] = msg.support;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected support");
  }

  if (!msg.description.empty())
  {
    j["description"] = msg.description.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_interfaces::msg::OptionalParameters object
///
/// \param j Reference to the JSON object containing serialized OptionalParameters data
/// \param msg Reference to the OptionalParameters message to populate
///
/// \throws std::runtime_error If failed to deserialize support field
inline void from_json(const nlohmann::json& j, OptionalParameters& msg)
{
  msg.parameter = j.at("parameter").get<std::string>();

  auto support = j.at("support").get<std::string>();
  if (support == "SUPPORTED" || support == "REQUIRED")
  {
    msg.support = support;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected support");
  }

  if (j.contains("description"))
  {
    msg.description.push_back(j.at("description").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_interfaces

#endif  // VDA5050_INTERFACES__JSON_UTILS__OPTIONAL_PARAMETERS_HPP_
