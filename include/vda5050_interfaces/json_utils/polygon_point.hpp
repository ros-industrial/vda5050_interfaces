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

#ifndef VDA5050_INTERFACES__JSON_UTILS__POLYGON_POINT_HPP_
#define VDA5050_INTERFACES__JSON_UTILS__POLYGON_POINT_HPP_

#include <nlohmann/json.hpp>

#include "vda5050_interfaces/msg/polygon_point.hpp"

namespace vda5050_interfaces {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_interfaces::msg::PolygonPoint object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the PolygonPoint message object to serialize
inline void to_json(nlohmann::json& j, const PolygonPoint& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_interfaces::msg::PolygonPoint object
///
/// \param j Reference to the JSON object containing serialized PolygonPoint data
/// \param msg Reference to the PolygonPoint message to populate
inline void from_json(const nlohmann::json& j, PolygonPoint& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
}

}  // namespace msg
}  // namespace vda5050_interfaces

#endif  // VDA5050_INTERFACES__JSON_UTILS__POLYGON_POINT_HPP_
