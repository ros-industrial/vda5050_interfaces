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

#ifndef VDA5050_MSGS__JSON_UTILS__MAX_STRING_LENS_HPP_
#define VDA5050_MSGS__JSON_UTILS__MAX_STRING_LENS_HPP_

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/max_string_lens.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::MaxStringLens object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the MaxStringLens message object to serialize
inline void to_json(nlohmann::json& j, const MaxStringLens& msg)
{
  if (!msg.msg_len.empty())
  {
    j["msgLen"] = msg.msg_len.front();
  }

  if (!msg.topic_serial_len.empty())
  {
    j["topicSerialLen"] = msg.topic_serial_len.front();
  }

  if (!msg.topic_elem_len.empty())
  {
    j["topicElemLen"] = msg.topic_elem_len.front();
  }

  if (!msg.id_len.empty())
  {
    j["idLen"] = msg.id_len.front();
  }

  if (!msg.enum_len.empty())
  {
    j["enumLen"] = msg.enum_len.front();
  }

  if (!msg.load_id_len.empty())
  {
    j["loadIdLen"] = msg.load_id_len.front();
  }

  if (!msg.id_numerical_only.empty())
  {
    j["idNumericalOnly"] = msg.id_numerical_only.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::MaxStringLens object
///
/// \param j Reference to the JSON object containing serialized MaxStringLens data
/// \param msg Reference to the MaxStringLens message to populate
inline void from_json(const nlohmann::json& j, MaxStringLens& msg)
{
  if (j.contains("msgLen"))
  {
    msg.msg_len.push_back(j.at("msgLen").get<uint32_t>());
  }

  if (j.contains("topicSerialLen"))
  {
    msg.topic_serial_len.push_back(j.at("topicSerialLen").get<uint32_t>());
  }

  if (j.contains("topicElemLen"))
  {
    msg.topic_elem_len.push_back(j.at("topicElemLen").get<uint32_t>());
  }

  if (j.contains("idLen"))
  {
    msg.id_len.push_back(j.at("idLen").get<uint32_t>());
  }

  if (j.contains("enumLen"))
  {
    msg.enum_len.push_back(j.at("enumLen").get<uint32_t>());
  }

  if (j.contains("loadIdLen"))
  {
    msg.load_id_len.push_back(j.at("loadIdLen").get<uint32_t>());
  }

  if (j.contains("idNumericalOnly"))
  {
    msg.id_numerical_only.push_back(j.at("idNumericalOnly").get<bool>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__MAX_STRING_LENS_HPP_
