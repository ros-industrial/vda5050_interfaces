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

#ifndef VDA5050_MSGS__JSON_UTILS__LOAD_SPECIFICATION_HPP_ 
#define VDA5050_MSGS__JSON_UTILS__LOAD_SPECIFICATION_HPP_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "vda5050_msgs/msg/load_specification.hpp"
#include "vda5050_msgs/json_utils/load_set.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::LoadSpecification object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the LoadSpecification message object to serialize
void to_json(nlohmann::json& j, const LoadSpecification& msg)
{
  if (!msg.load_positions.empty())
  {
    j["loadPositions"] = msg.load_positions;
  }

  if (!msg.load_sets.empty())
  {
    j["loadSets"] = msg.load_sets;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::LoadSpecification object
///
/// \param j Reference to the JSON object containing serialized LoadSpecification data
/// \param msg Reference to the LoadSpecification message to populate
void from_json(const nlohmann::json& j, LoadSpecification& msg)
{
  if (j.contains("loadPositions"))
  {
    msg.load_positions = j.at("loadPositions").get<std::vector<std::string>>();
  }

  if (j.contains("loadSets"))
  {
    msg.load_sets = j.at("loadSets").get<std::vector<LoadSet>>();
  }
}

} // namespace msg
} // namespace vda5050_msgs

#endif // VDA5050_MSGS__JSON_UTILS__LOAD_SPECIFICATION_HPP_
