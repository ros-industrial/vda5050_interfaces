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

#ifndef VDA5050_MSGS__JSON_UTILS__WHEEL_DEFINITION_HPP_
#define VDA5050_MSGS__JSON_UTILS__WHEEL_DEFINITION_HPP_

#include <nlohmann/json.hpp>
#include <string>

#include "vda5050_msgs/json_utils/position.hpp"
#include "vda5050_msgs/msg/wheel_definition.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::WheelDefinition object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the WheelDefinition message object to serialize
///
/// \throws std::runtime_error If failed to serialize type field
void to_json(nlohmann::json& j, const WheelDefinition& msg)
{
  if (
    msg.type == "DRIVE" || msg.type == "CASTER" || msg.type == "FIXED" ||
    msg.type == "MECANUM")
  {
    j["type"] = msg.type;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected type in wheel_definition");
  }

  j["isActiveDriven"] = msg.is_active_driven;
  j["isActiveSteered"] = msg.is_active_steered;
  j["position"] = msg.position;
  j["diameter"] = msg.diameter;
  j["width"] = msg.width;
  j["centerDisplacement"] = msg.center_displacement;

  if (!msg.constraints.empty())
  {
    j["constraints"] = msg.constraints.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::WheelDefinition object
///
/// \param j Reference to the JSON object containing serialized WheelDefinition data
/// \param msg Reference to the WheelDefinition message to populate
///
/// \throws std::runtime_error If failed to deserialize type field
void from_json(const nlohmann::json& j, WheelDefinition& msg)
{
  auto type = j.at("type").get<std::string>();
  if (
    type == "DRIVE" || type == "CASTER" || type == "FIXED" || type == "MECANUM")
  {
    msg.type = type;
  }
  else
  {
    throw std::runtime_error(
      "JSON parsing error: Unexpected type in wheel_definition");
  }

  msg.is_active_driven = j.at("isActiveDriven").get<bool>();
  msg.is_active_steered = j.at("isActiveSteered").get<bool>();
  msg.position = j.at("position").get<Position>();
  msg.diameter = j.at("diameter").get<double>();
  msg.width = j.at("width").get<double>();
  msg.center_displacement = j.at("centerDisplacement").get<double>();

  if (j.contains("constraints"))
  {
    msg.constraints.push_back(j.at("constraints").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__WHEEL_DEFINITION_HPP_
