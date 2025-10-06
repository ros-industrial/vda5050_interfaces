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

#ifndef VDA5050_MSGS__JSON_UTILS__SAFETY_STATE_HPP_
#define VDA5050_MSGS__JSON_UTILS__SAFETY_STATE_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/safety_state.hpp"

namespace vda5050_msgs {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::SafetyState object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize eStop
void to_json(nlohmann::json& j, const SafetyState& msg)
{
  if (
    msg.e_stop == SafetyState::E_STOP_AUTOACK ||
    msg.e_stop == SafetyState::E_STOP_MANUAL ||
    msg.e_stop == SafetyState::E_STOP_REMOTE ||
    msg.e_stop == SafetyState::E_STOP_NONE)
  {
    j["eStop"] = msg.e_stop;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected state for eStop");
  }

  j["fieldViolation"] = msg.field_violation;
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::SafetyState object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize eStop
void from_json(const nlohmann::json& j, SafetyState& msg)
{
  auto e_stop = j.at("eStop").get<std::string>();
  if (
    e_stop == SafetyState::E_STOP_AUTOACK ||
    e_stop == SafetyState::E_STOP_MANUAL ||
    e_stop == SafetyState::E_STOP_REMOTE || e_stop == SafetyState::E_STOP_NONE)
  {
    msg.e_stop = e_stop;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected state for eStop");
  }

  msg.field_violation = j.at("fieldViolation").get<bool>();
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__SAFETY_STATE_HPP_
