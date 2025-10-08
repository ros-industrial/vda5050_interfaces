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

#ifndef VDA5050_MSGS__JSON_UTILS__TIMING_HPP_
#define VDA5050_MSGS__JSON_UTILS__TIMING_HPP_

#include "nlohmann/json.hpp"

#include "vda5050_msgs/msg/timing.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Timing object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Timing message object to serialize
void to_json(nlohmann::json& j, const Timing& msg)
{
  j["minOrderInterval"] = msg.min_order_interval;
  j["minStateInterval"] = msg.min_state_interval;

  if (!msg.default_state_interval.empty())
  {
    j["defaultStateInterval"] = msg.default_state_interval.front();
  }

  if (!msg.visualization_interval.empty())
  {
    j["visualizationInterval"] = msg.visualization_interval.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Timing object
///
/// \param j Reference to the JSON object containing serialized Timing data
/// \param msg Reference to the Timing message to populate
void from_json(const nlohmann::json& j, Timing& msg)
{
  msg.min_order_interval = j.at("minOrderInterval").get<double>();
  msg.min_state_interval = j.at("minStateInterval").get<double>();

  if (j.contains("defaultStateInterval"))
  {
    msg.default_state_interval.push_back(
      j.at("defaultStateInterval").get<double>());
  }

  if (j.contains("visualizationInterval"))
  {
    msg.visualization_interval.push_back(
      j.at("visualizationInterval").get<double>());
  }
}

} // namespace msg
} // namespace vda5050_msgs

#endif // VDA5050_MSGS__JSON_UTILS__TIMING_HPP_
