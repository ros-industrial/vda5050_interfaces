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

#ifndef VDA5050_MSGS__JSON_UTILS__PROTOCOL_LIMITS_HPP_
#define VDA5050_MSGS__JSON_UTILS__PROTOCOL_LIMITS_HPP_

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/max_array_lens.hpp"
#include "vda5050_msgs/json_utils/max_string_lens.hpp"
#include "vda5050_msgs/json_utils/timing.hpp"
#include "vda5050_msgs/msg/protocol_limits.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::ProtocolLimits object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the ProtocolLimits message object to serialize
inline void to_json(nlohmann::json& j, const ProtocolLimits& msg)
{
  j["maxStringLens"] = msg.max_string_lens;
  j["maxArrayLens"] = msg.max_array_lens;
  j["timing"] = msg.timing;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::ProtocolLimits object
///
/// \param j Reference to the JSON object containing serialized ProtocolLimits data
/// \param msg Reference to the ProtocolLimits message to populate
inline void from_json(const nlohmann::json& j, ProtocolLimits& msg)
{
  msg.max_string_lens = j.at("maxStringLens").get<MaxStringLens>();
  msg.max_array_lens = j.at("maxArrayLens").get<MaxArrayLens>();
  msg.timing = j.at("timing").get<Timing>();
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__PROTOCOL_LIMITS_HPP_
