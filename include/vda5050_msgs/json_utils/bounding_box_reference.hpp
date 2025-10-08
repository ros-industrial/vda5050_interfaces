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

#ifndef VDA5050_MSGS__JSON_UTILS__BOUNDING_BOX_REFERENCE_HPP_
#define VDA5050_MSGS__JSON_UTILS__BOUNDING_BOX_REFERENCE_HPP_

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/bounding_box_reference.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::BoundingBoxReference object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const BoundingBoxReference& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["z"] = msg.z;

  if (!msg.theta.empty())
  {
    j["theta"] = msg.theta.front();
  }
}

// ============================================================================
/// \brief Populate a vda5050_msgs::msg::BoundingBoxReference object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, BoundingBoxReference& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.z = j.at("z").get<double>();

  if (j.contains("theta"))
  {
    msg.theta.push_back(j.at("theta").get<double>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__BOUNDING_BOX_REFERENCE_HPP_
