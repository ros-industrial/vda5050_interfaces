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

#ifndef VDA5050_MSGS__JSON_UTILS__EDGE_HPP_
#define VDA5050_MSGS__JSON_UTILS__EDGE_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/action.hpp"
#include "vda5050_msgs/json_utils/trajectory.hpp"
#include "vda5050_msgs/msg/edge.hpp"

namespace vda5050_msgs {

namespace msg {
//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Edge object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize orientationType
void to_json(nlohmann::json& j, const Edge& msg)
{
  j["edgeId"] = msg.edge_id;
  j["sequenceId"] = msg.sequence_id;
  j["startNodeId"] = msg.start_node_id;
  j["endNodeId"] = msg.end_node_id;
  j["released"] = msg.released;
  j["actions"] = msg.actions;

  if (!msg.edge_description.empty())
  {
    j["edgeDescription"] = msg.edge_description.front();
  }

  if (!msg.max_speed.empty())
  {
    j["maxSpeed"] = msg.max_speed.front();
  }

  if (!msg.max_height.empty())
  {
    j["maxHeight"] = msg.max_height.front();
  }

  if (!msg.min_height.empty())
  {
    j["minHeight"] = msg.min_height.front();
  }

  if (!msg.orientation.empty())
  {
    j["orientation"] = msg.orientation.front();
  }

  if (
    msg.orientation_type == msg.ORIENTATION_TYPE_TANGENTIAL ||
    msg.orientation_type == msg.ORIENTATION_TYPE_GLOBAL)
  {
    j["orientationType"] = msg.orientation_type;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected orientationType.");
  }

  if (!msg.direction.empty())
  {
    j["direction"] = msg.direction.front();
  }

  if (!msg.rotation_allowed.empty())
  {
    j["rotationAllowed"] = msg.rotation_allowed.front();
  }

  if (!msg.max_rotation_speed.empty())
  {
    j["maxRotationSpeed"] = msg.max_rotation_speed.front();
  }

  if (!msg.trajectory.empty())
  {
    j["trajectory"] = msg.trajectory.front();
  }

  if (!msg.length.empty())
  {
    j["length"] = msg.length.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Edge object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize orientationType
void from_json(const nlohmann::json& j, Edge& msg)
{
  msg.edge_id = j.at("edgeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<int32_t>();
  msg.start_node_id = j.at("startNodeId").get<std::string>();
  msg.end_node_id = j.at("endNodeId").get<std::string>();
  msg.released = j.at("released").get<bool>();
  msg.actions = j.at("actions").get<std::vector<Action>>();

  if (j.contains("edgeDescription"))
  {
    msg.edge_description.push_back(j.at("edgeDescription").get<std::string>());
  }

  if (j.contains("maxSpeed"))
  {
    msg.max_speed.push_back(j.at("maxSpeed").get<double>());
  }

  if (j.contains("maxHeight"))
  {
    msg.max_height.push_back(j.at("maxHeight").get<double>());
  }

  if (j.contains("minHeight"))
  {
    msg.min_height.push_back(j.at("minHeight").get<double>());
  }

  if (j.contains("orientation"))
  {
    msg.orientation.push_back(j.at("orientation").get<double>());
  }

  auto orientation_type = j.at("orientationType").get<std::string>();
  if (
    orientation_type == Edge::ORIENTATION_TYPE_TANGENTIAL ||
    orientation_type == Edge::ORIENTATION_TYPE_GLOBAL)
  {
    msg.orientation_type = orientation_type;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected orientationType.");
  }

  if (j.contains("direction"))
  {
    msg.direction.push_back(j.at("direction").get<std::string>());
  }

  if (j.contains("rotationAllowed"))
  {
    msg.rotation_allowed.push_back(j.at("rotationAllowed").get<bool>());
  }

  if (j.contains("maxRotationSpeed"))
  {
    msg.max_rotation_speed.push_back(j.at("maxRotationSpeed").get<double>());
  }

  if (j.contains("trajectory"))
  {
    msg.trajectory.push_back(j.at("trajectory").get<Trajectory>());
  }

  if (j.contains("length"))
  {
    msg.length.push_back(j.at("length").get<double>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__EDGE_HPP_
