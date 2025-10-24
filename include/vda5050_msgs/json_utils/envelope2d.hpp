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

#ifndef VDA5050_MSGS__JSON_UTILS__ENVELOPE2D_HPP_
#define VDA5050_MSGS__JSON_UTILS__ENVELOPE2D_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/polygon_point.hpp"
#include "vda5050_msgs/msg/envelope2d.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Envelope2d object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Envelope2d message object to serialize
void to_json(nlohmann::json& j, const Envelope2d& msg)
{
  j["set"] = msg.set;
  j["polygonPoints"] = msg.polygon_points;

  if (!msg.description.empty())
  {
    j["description"] = msg.description.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Envelope2d object
///
///\param j Reference to the JSON object containing serialized Envelope2d data
/// \param msg Reference to the Envelope2d message to populate
void from_json(const nlohmann::json& j, Envelope2d& msg)
{
  msg.set = j.at("set").get<std::string>();
  msg.polygon_points = j.at("polygonPoints").get<std::vector<PolygonPoint>>();

  if (j.contains("description"))
  {
    msg.description.push_back(j.at("description").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__ENVELOPE2D_HPP_
