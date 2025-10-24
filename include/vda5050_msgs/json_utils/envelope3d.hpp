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

#ifndef VDA5050_MSGS__JSON_UTILS__ENVELOPE3D_HPP_
#define VDA5050_MSGS__JSON_UTILS__ENVELOPE3D_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/envelope3d.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Envelope3d object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Envelope3d message object to serialize
void to_json(nlohmann::json& j, const Envelope3d& msg)
{
  j["set"] = msg.set;
  j["format"] = msg.format;

  if (!msg.data.empty())
  {
    j["data"] = msg.data.front();
  }

  if (!msg.url.empty())
  {
    j["url"] = msg.url.front();
  }

  if (!msg.description.empty())
  {
    j["description"] = msg.description.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Envelope3d object
///
/// \param j Reference to the JSON object containing serialized Envelope3d data
/// \param msg Reference to the Envelope3d message to populate
void from_json(const nlohmann::json& j, Envelope3d& msg)
{
  msg.set = j.at("set").get<std::string>();
  msg.format = j.at("format").get<std::string>();

  if (j.contains("data"))
  {
    msg.data.push_back(j.at("data").get<std::string>());
  }

  if (j.contains("url"))
  {
    msg.url.push_back(j.at("url").get<std::string>());
  }

  if (j.contains("description"))
  {
    msg.description.push_back(j.at("description").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__ENVELOPE3D_HPP_
