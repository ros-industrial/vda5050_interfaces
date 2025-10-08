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

#ifndef VDA5050_MSGS__JSON_UTILS__AGV_ACTION_HPP_
#define VDA5050_MSGS__JSON_UTILS__AGV_ACTION_HPP_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "vda5050_msgs/json_utils/factsheet_action_parameter.hpp"
#include "vda5050_msgs/msg/agv_action.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::AgvAction object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize action_scopes or blocking_types
void to_json(nlohmann::json& j, const AgvAction& msg)
{
  j["actionType"] = msg.action_type;

  for (std::string scope : msg.action_scopes)
  {
    if (scope != "INSTANT" && scope != "NODE" && scope != "EDGE")
    {
      throw std::runtime_error(
        "Serialization error: Unexpected scope in action_scopes");
    }
  }
  j["actionScopes"] = msg.action_scopes;

  if (!msg.factsheet_action_parameters.empty())
  {
    j["factsheetActionParameters"] = msg.factsheet_action_parameters;
  }

  if (!msg.result_description.empty())
  {
    j["resultDescription"] = msg.result_description.front();
  }

  if (!msg.action_description.empty())
  {
    j["actionDescription"] = msg.action_description.front();
  }

  if (!msg.blocking_types.empty())
  {
    for (std::string type : msg.blocking_types)
    {
      if (type != "NONE" && type != "SOFT" && type != "HARD")
      {
        throw std::runtime_error(
          "Serialization error: Unexpected type in blocking_types");
      }
    }

    j["blockingTypes"] = msg.blocking_types;
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::AgvAction object
///
/// \param j Reference to the JSON object containing serialized AgvAction data
/// \param msg Reference to the AgvAction message to populate
///
/// \throws std::runtime_error If failed to deserialize action_scopes or blocking_types
void from_json(const nlohmann::json& j, AgvAction& msg)
{
  msg.action_type = j.at("actionType").get<std::string>();

  auto action_scopes = j.at("actionScopes").get<std::vector<std::string>>();
  for (std::string scope : action_scopes)
  {
    if (scope != "INSTANT" && scope != "NODE" && scope != "EDGE")
    {
      throw std::runtime_error(
        "JSON parsing error: Unexpected scope in action_scopes");
    }
  }
  msg.action_scopes = action_scopes;

  if (j.contains("factsheetActionParameters"))
  {
    msg.factsheet_action_parameters =
      j.at("factsheetActionParameters")
        .get<std::vector<FactsheetActionParameter>>();
  }

  if (j.contains("resultDescription"))
  {
    msg.result_description.push_back(
      j.at("resultDescription").get<std::string>());
  }

  if (j.contains("actionDescription"))
  {
    msg.action_description.push_back(
      j.at("actionDescription").get<std::string>());
  }

  if (j.contains("blockingTypes"))
  {
    auto blocking_types = j.at("blockingTypes").get<std::vector<std::string>>();
    for (std::string type : blocking_types)
    {
      if (type != "NONE" && type != "SOFT" && type != "HARD")
      {
        throw std::runtime_error(
          "JSON parsing error: Unexpected type in blocking_types");
      }
    }
    msg.blocking_types = blocking_types;
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__AGV_ACTION_HPP_
