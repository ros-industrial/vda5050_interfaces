/**
 * Copyright (C) 2025 ROS Industrial Consortium Asia Pacific
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

#ifndef VDA5050_MSGS__JSON_UTILS__CONNECTION_HPP
#define VDA5050_MSGS__JSON_UTILS__CONNECTION_HPP

#include <nlohmann/json.hpp>

#include "vda5050_msgs/msg/connection.hpp"

namespace vda5050_msgs {

namespace json_utils {

/// \brief Convert a vda5050_msgs::msg::Connection object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const msg::Connection& msg);

/// \brief Populate a vda5050_msgs::msg::Connection object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized connection data
/// \param msg Reference to the Connection message to populate
void from_json(nlohmann::json& j, msg::Connection& msg);

}  // namespace json_utils
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__CONNECTION_HPP

