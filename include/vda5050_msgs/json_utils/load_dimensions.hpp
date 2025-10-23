/*
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

#ifndef VDA5050_MSGS__JSON_UTILS__LOAD_DIMENSIONS_HPP_
#define VDA5050_MSGS__JSON_UTILS__LOAD_DIMENSIONS_HPP_

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/load_dimensions.hpp"

namespace vda5050_msgs {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::LoadDimensions object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
inline void to_json(nlohmann::json& j, const LoadDimensions& msg)
{
  j["length"] = msg.length;
  j["width"] = msg.width;

  if (!msg.height.empty())
  {
    j["height"] = msg.height.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::LoadDimensions object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
inline void from_json(const nlohmann::json& j, LoadDimensions& msg)
{
  msg.length = j.at("length").get<double>();
  msg.width = j.at("width").get<double>();

  if (j.contains("height"))
  {
    msg.height.push_back(j.at("height").get<double>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__LOAD_DIMENSIONS_HPP_
