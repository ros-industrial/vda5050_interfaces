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

#ifndef VDA5050_MSGS__JSON_UTILS__INFO_HPP_
#define VDA5050_MSGS__JSON_UTILS__INFO_HPP_

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/info_reference.hpp"
#include "vda5050_msgs/msg/info.hpp"
#include "vda5050_msgs/msg/info_reference.hpp"

namespace vda5050_msgs {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Info object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize infoLevel
void to_json(nlohmann::json& j, const Info& msg)
{
  j["infoType"] = msg.info_type;

  if (
    msg.info_level == Info::INFO_LEVEL_INFO ||
    msg.info_level == Info::INFO_LEVEL_DEBUG)
  {
    j["infoLevel"] = msg.info_level;
  }
  else
  {
    throw std::runtime_error("Serialiation error: Unexpected infoLevel");
  }

  if (!msg.info_references.empty())
  {
    j["infoReferences"] = msg.info_references;
  }

  if (!msg.info_description.empty())
  {
    j["infoDescription"] = msg.info_description.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Info object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize infoLevel
void from_json(const nlohmann::json& j, Info& msg)
{
  msg.info_type = j.at("infoType").get<std::string>();

  auto info_level = j.at("infoLevel").get<std::string>();
  if (
    info_level == Info::INFO_LEVEL_INFO || info_level == Info::INFO_LEVEL_DEBUG)
  {
    msg.info_level = info_level;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected infoLevel");
  }

  if (j.contains("infoReferences"))
  {
    msg.info_references =
      j.at("infoReferences").get<std::vector<InfoReference>>();
  }

  if (j.contains("infoDescription"))
  {
    msg.info_description.push_back(j.at("infoDescription").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__INFO_HPP_
