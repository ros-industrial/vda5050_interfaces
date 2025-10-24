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

#ifndef VDA5050_MSGS__JSON_UTILS__LOAD_SET_HPP_
#define VDA5050_MSGS__JSON_UTILS__LOAD_SET_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/bounding_box_reference.hpp"
#include "vda5050_msgs/json_utils/load_dimensions.hpp"
#include "vda5050_msgs/msg/load_set.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::LoadSet object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the LoadSet message object to serialize
inline void to_json(nlohmann::json& j, const LoadSet& msg)
{
  j["setName"] = msg.set_name;
  j["loadType"] = msg.load_type;

  if (!msg.load_positions.empty())
  {
    j["loadPositions"] = msg.load_positions;
  }

  if (!msg.bounding_box_reference.empty())
  {
    j["boundingBoxReference"] = msg.bounding_box_reference.front();
  }

  if (!msg.load_dimensions.empty())
  {
    j["loadDimensions"] = msg.load_dimensions.front();
  }

  if (!msg.max_weight.empty())
  {
    j["maxWeight"] = msg.max_weight.front();
  }

  if (!msg.min_load_handling_height.empty())
  {
    j["minLoadHandlingHeight"] = msg.min_load_handling_height.front();
  }

  if (!msg.max_load_handling_height.empty())
  {
    j["maxLoadHandlingHeight"] = msg.max_load_handling_height.front();
  }

  if (!msg.min_load_handling_depth.empty())
  {
    j["minLoadHandlingDepth"] = msg.min_load_handling_depth.front();
  }

  if (!msg.max_load_handling_depth.empty())
  {
    j["maxLoadHandlingDepth"] = msg.max_load_handling_depth.front();
  }

  if (!msg.min_load_handling_tilt.empty())
  {
    j["minLoadHandlingTilt"] = msg.min_load_handling_tilt.front();
  }

  if (!msg.max_load_handling_tilt.empty())
  {
    j["maxLoadHandlingTilt"] = msg.max_load_handling_tilt.front();
  }

  if (!msg.agv_speed_limit.empty())
  {
    j["agvSpeedLimit"] = msg.agv_speed_limit.front();
  }

  if (!msg.agv_acceleration_limit.empty())
  {
    j["agvAccelerationLimit"] = msg.agv_acceleration_limit.front();
  }

  if (!msg.agv_deceleration_limit.empty())
  {
    j["agvDecelerationLimit"] = msg.agv_deceleration_limit.front();
  }

  if (!msg.pick_time.empty())
  {
    j["pickTime"] = msg.pick_time.front();
  }

  if (!msg.drop_time.empty())
  {
    j["dropTime"] = msg.drop_time.front();
  }

  if (!msg.description.empty())
  {
    j["description"] = msg.description.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::LoadSet object
///
/// \param j Reference to the JSON object containing serialized LoadSet data
/// \param msg Reference to the LoadSet message to populate
inline void from_json(const nlohmann::json& j, LoadSet& msg)
{
  msg.set_name = j.at("setName").get<std::string>();
  msg.load_type = j.at("loadType").get<std::string>();

  if (j.contains("loadPositions"))
  {
    msg.load_positions = j.at("loadPositions").get<std::vector<std::string>>();
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

  if (j.contains("maxWeight"))
  {
    msg.max_weight.push_back(j.at("maxWeight").get<double>());
  }

  if (j.contains("minLoadHandlingHeight"))
  {
    msg.min_load_handling_height.push_back(
      j.at("minLoadHandlingHeight").get<double>());
  }

  if (j.contains("maxLoadHandlingHeight"))
  {
    msg.max_load_handling_height.push_back(
      j.at("maxLoadHandlingHeight").get<double>());
  }

  if (j.contains("minLoadHandlingDepth"))
  {
    msg.min_load_handling_depth.push_back(
      j.at("minLoadHandlingDepth").get<double>());
  }

  if (j.contains("maxLoadHandlingDepth"))
  {
    msg.max_load_handling_depth.push_back(
      j.at("maxLoadHandlingDepth").get<double>());
  }

  if (j.contains("minLoadHandlingTilt"))
  {
    msg.min_load_handling_tilt.push_back(
      j.at("minLoadHandlingTilt").get<double>());
  }

  if (j.contains("maxLoadHandlingTilt"))
  {
    msg.max_load_handling_tilt.push_back(
      j.at("maxLoadHandlingTilt").get<double>());
  }

  if (j.contains("agvSpeedLimit"))
  {
    msg.agv_speed_limit.push_back(j.at("agvSpeedLimit").get<double>());
  }

  if (j.contains("agvAccelerationLimit"))
  {
    msg.agv_acceleration_limit.push_back(
      j.at("agvAccelerationLimit").get<double>());
  }

  if (j.contains("agvDecelerationLimit"))
  {
    msg.agv_deceleration_limit.push_back(
      j.at("agvDecelerationLimit").get<double>());
  }

  if (j.contains("pickTime"))
  {
    msg.pick_time.push_back(j.at("pickTime").get<double>());
  }

  if (j.contains("dropTime"))
  {
    msg.drop_time.push_back(j.at("dropTime").get<double>());
  }

  if (j.contains("description"))
  {
    msg.description.push_back(j.at("description").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__LOAD_SET_HPP_
