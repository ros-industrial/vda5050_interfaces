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

#ifndef VDA5050_MSGS__JSON_UTILS__PHYSICAL_PARAMETERS_HPP_
#define VDA5050_MSGS__JSON_UTILS__PHYSICAL_PARAMETERS_HPP_

#include <nlohmann/json.hpp>
#include <string>

#include "vda5050_msgs/msg/physical_parameters.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::PhysicalParameters object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the PhysicalParameters message object to serialize
void to_json(nlohmann::json& j, const PhysicalParameters& msg)
{
  j["speedMin"] = msg.speed_min;
  j["speedMax"] = msg.speed_max;
  j["accelerationMax"] = msg.acceleration_max;
  j["decelerationMax"] = msg.deceleration_max;
  j["heightMin"] = msg.height_min;
  j["heightMax"] = msg.height_max;
  j["width"] = msg.width;
  j["length"] = msg.length;

  if (!msg.angular_speed_min.empty())
  {
    j["angularSpeedMin"] = msg.angular_speed_min.front();
  }

  if (!msg.angular_speed_max.empty())
  {
    j["angularSpeedMax"] = msg.angular_speed_max.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::PhysicalParameters object
///
/// \param j Reference to the JSON object containing serialized PhysicalParameters data
/// \param msg Reference to the PhysicalParameters message to populate
void from_json(const nlohmann::json& j, PhysicalParameters& msg)
{
  msg.speed_min = j.at("speedMin").get<double>();
  msg.speed_max = j.at("speedMax").get<double>();
  msg.acceleration_max = j.at("accelerationMax").get<double>();
  msg.deceleration_max = j.at("decelerationMax").get<double>();
  msg.height_min = j.at("heightMin").get<double>();
  msg.height_max = j.at("heightMax").get<double>();
  msg.width = j.at("width").get<double>();
  msg.length = j.at("length").get<double>();

  if (j.contains("angularSpeedMin"))
  {
    msg.angular_speed_min.push_back(j.at("angularSpeedMin").get<double>());
  }

  if (j.contains("angularSpeedMax"))
  {
    msg.angular_speed_max.push_back(j.at("angularSpeedMax").get<double>());
  }
}


} // namespace msg
} // namespace vda5050_msgs

#endif //VDA5050_MSGS__JSON_UTILS__PHYSICAL_PARAMETERS_HPP_
