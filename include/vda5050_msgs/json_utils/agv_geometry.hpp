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

#ifndef VDA5050_MSGS__JSON_UTILS__AGV_GEOMETRY_HPP_
#define VDA5050_MSGS__JSON_UTILS__AGV_GEOMETRY_HPP_

#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/envelope2d.hpp"
#include "vda5050_msgs/json_utils/envelope3d.hpp"
#include "vda5050_msgs/json_utils/wheel_definition.hpp"
#include "vda5050_msgs/msg/agv_geometry.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::AGVGeometry object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the AGVGeometry message object to serialize
inline void to_json(nlohmann::json& j, const AGVGeometry& msg)
{
  if (!msg.wheel_definitions.empty())
  {
    j["wheelDefinitions"] = msg.wheel_definitions;
  }

  if (!msg.envelopes_2d.empty())
  {
    j["envelopes2d"] = msg.envelopes_2d;
  }

  if (!msg.envelopes_3d.empty())
  {
    j["envelopes3d"] = msg.envelopes_3d;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::AGVGeometry object
///
/// \param j Reference to the JSON object containing serialized AGVGeometry data
/// \param msg Reference to the AGVGeometry message to populate
inline void from_json(const nlohmann::json& j, AGVGeometry& msg)
{
  if (j.contains("wheelDefinitions"))
  {
    msg.wheel_definitions =
      j.at("wheelDefinitions").get<std::vector<WheelDefinition>>();
  }

  if (j.contains("envelopes2d"))
  {
    msg.envelopes_2d = j.at("envelopes2d").get<std::vector<Envelope2d>>();
  }

  if (j.contains("envelopes3d"))
  {
    msg.envelopes_3d = j.at("envelopes3d").get<std::vector<Envelope3d>>();
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__AGV_GEOMETRY_HPP_
