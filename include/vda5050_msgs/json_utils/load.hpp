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

#ifndef VDA5050_MSGS__JSON_UTILS__LOAD_HPP_
#define VDA5050_MSGS__JSON_UTILS__LOAD_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/bounding_box_reference.hpp"
#include "vda5050_msgs/json_utils/load_dimensions.hpp"
#include "vda5050_msgs/msg/load.hpp"

namespace vda5050_msgs {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Load object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const Load& msg)
{
  if (!msg.load_id.empty())
  {
    j["loadId"] = msg.load_id.front();
  }

  if (!msg.load_type.empty())
  {
    j["loadType"] = msg.load_type.front();
  }

  if (!msg.load_position.empty())
  {
    j["loadPosition"] = msg.load_position.front();
  }

  if (!msg.bounding_box_reference.empty())
  {
    j["boundingBoxReference"] = msg.bounding_box_reference.front();
  }

  if (!msg.load_dimensions.empty())
  {
    j["loadDimensions"] = msg.load_dimensions.front();
  }

  if (!msg.weight.empty())
  {
    j["weight"] = msg.weight.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Load object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, Load& msg)
{
  if (j.contains("loadId"))
  {
    msg.load_id.push_back(j.at("loadId").get<std::string>());
  }

  if (j.contains("loadType"))
  {
    msg.load_type.push_back(j.at("loadType").get<std::string>());
  }

  if (j.contains("loadPosition"))
  {
    msg.load_position.push_back(j.at("loadPosition").get<std::string>());
  }

  if (j.contains("boundingBoxReference"))
  {
    msg.bounding_box_reference.push_back(
      j.at("boundingBoxReference").get<BoundingBoxReference>());
  }

  if (j.contains("loadDimensions"))
  {
    msg.load_dimensions.push_back(j.at("loadDimensions").get<LoadDimensions>());
  }

  if (j.contains("weight"))
  {
    msg.weight.push_back(j.at("weight").get<double>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__LOAD_HPP_
