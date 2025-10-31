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

#ifndef VDA5050_INTERFACES__JSON_UTILS__EDGE_STATE_HPP_
#define VDA5050_INTERFACES__JSON_UTILS__EDGE_STATE_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_interfaces/msg/edge_state.hpp"
#include "vda5050_interfaces/msg/trajectory.hpp"

#include "vda5050_interfaces/json_utils/trajectory.hpp"

namespace vda5050_interfaces {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_interfaces::msg::EdgeState object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
inline void to_json(nlohmann::json& j, const EdgeState& msg)
{
  j["edgeId"] = msg.edge_id;
  j["sequenceId"] = msg.sequence_id;
  j["released"] = msg.released;

  if (!msg.edge_description.empty())
  {
    j["edgeDescription"] = msg.edge_description.front();
  }

  if (!msg.trajectory.empty())
  {
    j["trajectory"] = msg.trajectory.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_interfaces::msg::EdgeState object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
inline void from_json(const nlohmann::json& j, EdgeState& msg)
{
  msg.edge_id = j.at("edgeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<uint32_t>();
  msg.released = j.at("released").get<bool>();

  if (j.contains("edgeDescription"))
  {
    msg.edge_description.push_back(j.at("edgeDescription").get<std::string>());
  }

  if (j.contains("trajectory"))
  {
    msg.trajectory.push_back(j.at("trajectory").get<Trajectory>());
  }
}

}  // namespace msg
}  // namespace vda5050_interfaces

#endif  // VDA5050_INTERFACES__JSON_UTILS__EDGE_STATE_HPP_
