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

#ifndef VDA5050_INTERFACES__JSON_UTILS__VISUALIZATION_HPP_
#define VDA5050_INTERFACES__JSON_UTILS__VISUALIZATION_HPP_

#include <nlohmann/json.hpp>

#include "vda5050_interfaces/msg/header.hpp"
#include "vda5050_interfaces/msg/visualization.hpp"

#include "vda5050_interfaces/json_utils/agv_position.hpp"
#include "vda5050_interfaces/json_utils/velocity.hpp"

namespace vda5050_interfaces {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_interfaces::msg::Visualization object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
inline void to_json(nlohmann::json& j, const Visualization& msg)
{
  to_json(j, msg.header);

  if (!msg.agv_position.empty())
  {
    j["agvPosition"] = msg.agv_position.front();
  }

  if (!msg.velocity.empty())
  {
    j["velocity"] = msg.velocity.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_interfaces::msg::Visualization object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
inline void from_json(const nlohmann::json& j, Visualization& msg)
{
  from_json(j, msg.header);

  if (j.contains("agvPosition"))
  {
    msg.agv_position.push_back(j.at("agvPosition").get<AGVPosition>());
  }

  if (j.contains("velocity"))
  {
    msg.velocity.push_back(j.at("velocity").get<Velocity>());
  }
}

}  // namespace msg
}  // namespace vda5050_interfaces

#endif  // VDA5050_INTERFACES__JSON_UTILS__VISUALIZATION_HPP_
