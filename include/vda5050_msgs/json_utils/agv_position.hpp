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

#ifndef VDA5050_MSGS__JSON_UTILS__AGV_POSITION_HPP_
#define VDA5050_MSGS__JSON_UTILS__AGV_POSITION_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/agv_position.hpp"

namespace vda5050_msgs {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::AGVPosition object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
inline void to_json(nlohmann::json& j, const AGVPosition& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["theta"] = msg.theta;
  j["mapId"] = msg.map_id;
  j["positionInitialized"] = msg.position_initialized;

  if (!msg.map_description.empty())
  {
    j["mapDescription"] = msg.map_description.front();
  }

  if (!msg.localization_score.empty())
  {
    j["localizationScore"] = msg.localization_score.front();
  }

  if (!msg.deviation_range.empty())
  {
    j["deviationRange"] = msg.deviation_range.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::AGVPosition object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
inline void from_json(const nlohmann::json& j, AGVPosition& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.theta = j.at("theta").get<double>();
  msg.map_id = j.at("mapId").get<std::string>();
  msg.position_initialized = j.at("positionInitialized").get<bool>();

  if (j.contains("mapDescription"))
  {
    msg.map_description.push_back(j.at("mapDescription").get<std::string>());
  }

  if (j.contains("localizationScore"))
  {
    msg.localization_score.push_back(j.at("localizationScore").get<double>());
  }

  if (j.contains("deviationRange"))
  {
    msg.deviation_range.push_back(j.at("deviationRange").get<double>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__AGV_POSITION_HPP_
