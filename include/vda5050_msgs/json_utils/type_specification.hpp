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

#ifndef VDA5050_MSGS__JSON_UTILS__TYPE_SPECIFICATION_HPP_
#define VDA5050_MSGS__JSON_UTILS__TYPE_SPECIFICATION_HPP_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "vda5050_msgs/msg/type_specification.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::TypeSpecification object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the TypeSpecification message object to serialize
///
/// \throws std::runtime_error If failed to serialize agv_kinematic or agv_class field
void to_json(nlohmann::json& j, const TypeSpecification& msg)
{
  j["seriesName"] = msg.series_name;

  if (
    msg.agv_kinematic == "DIFF" || msg.agv_kinematic == "OMNI" ||
    msg.agv_kinematic == "THREEWHEEL")
  {
    j["agvKinematic"] = msg.agv_kinematic;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected agv_kinematic");
  }

  if (
    msg.agv_class == "FORKLIFT" || msg.agv_class == "CONVEYOR" ||
    msg.agv_class == "TUGGER" || msg.agv_class == "CARRIER")
  {
    j["agvClass"] = msg.agv_class;
  }
  else
  {
    throw std::runtime_error("Serialization error: Unexpected agv_class");
  }

  j["maxLoadMass"] = msg.max_load_mass;
  j["localizationTypes"] = msg.localization_types;
  j["navigationTypes"] = msg.navigation_types;

  if (!msg.series_description.empty())
  {
    j["seriesDescription"] = msg.series_description.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::TypeSpecification object
///
/// \param j Reference to the JSON object containing serialized TypeSpecification data
/// \param msg Reference to the TypeSpecification message to populate
///
/// \throws std::runtime_error If failed to deserialize agv_kinematic or agv_class field
void from_json(const nlohmann::json& j, TypeSpecification& msg)
{
  msg.series_name = j.at("seriesName").get<std::string>();

  auto agv_kinematic = j.at("agvKinematic").get<std::string>();
  if (
    agv_kinematic == "DIFF" || agv_kinematic == "OMNI" ||
    agv_kinematic == "THREEWHEEL")
  {
    msg.agv_kinematic = agv_kinematic;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected agv_kinematic");
  }

  auto agv_class = j.at("agvClass").get<std::string>();
  if (
    agv_class == "FORKLIFT" || agv_class == "CONVEYOR" ||
    agv_class == "TUGGER" || agv_class == "CARRIER")
  {
    msg.agv_class = agv_class;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected agv_class");
  }

  msg.max_load_mass = j.at("maxLoadMass").get<double>();
  msg.localization_types =
    j.at("localizationTypes").get<std::vector<std::string>>();
  msg.navigation_types =
    j.at("navigationTypes").get<std::vector<std::string>>();

  if (j.contains("seriesDescription"))
  {
    msg.series_description.push_back(
      j.at("seriesDescription").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__TYPE_SPECIFICATION_HPP_
