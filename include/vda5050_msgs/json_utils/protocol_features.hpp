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

#ifndef VDA5050_MSGS__JSON_UTILS__PROTOCOL_FEATURES_HPP_
#define VDA5050_MSGS__JSON_UTILS__PROTOCOL_FEATURES_HPP_

#include <nlohmann/json.hpp>
#include <vector>

#include "vda5050_msgs/json_utils/agv_action.hpp"
#include "vda5050_msgs/json_utils/optional_parameters.hpp"
#include "vda5050_msgs/msg/protocol_features.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::ProtocolFeatures object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the ProtocolFeatures message object to serialize
void to_json(nlohmann::json& j, const ProtocolFeatures& msg)
{
  j["optionalParameters"] = msg.optional_parameters;
  j["agvActions"] = msg.agv_actions;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::ProtocolFeatures object
///
/// \param j Reference to the JSON object containing serialized ProtocolFeatures data
/// \param msg Reference to the ProtocolFeatures message to populate
void from_json(const nlohmann::json& j, ProtocolFeatures& msg)
{
  msg.optional_parameters =
    j.at("optionalParameters").get<std::vector<OptionalParameters>>();
  msg.agv_actions = j.at("agvActions").get<std::vector<AgvAction>>();
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__PROTOCOL_FEATURES_HPP_
