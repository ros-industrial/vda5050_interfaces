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

#ifndef VDA5050_MSGS__JSON_UTILS__CONNECTION_HPP_
#define VDA5050_MSGS__JSON_UTILS__CONNECTION_HPP_

#include <nlohmann/json.hpp>
#include <string>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/msg/connection.hpp"

namespace vda5050_msgs {

namespace msg {

/// \brief Convert a vda5050_msgs::msg::Connection object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize connection_state
void to_json(nlohmann::json& j, const Connection& msg)
{
  to_json(j, msg.header);

  if (
    msg.connection_state == Connection::CONNECTION_STATE_ONLINE ||
    msg.connection_state == Connection::CONNECTION_STATE_OFFLINE ||
    msg.connection_state == Connection::CONNECTION_STATE_CONNECTIONBROKEN)
  {
    j["connectionState"] = msg.connection_state;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected connection state");
  }
}

/// \brief Populate a vda5050_msgs::msg::Connection object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized connection data
/// \param msg Reference to the Connection message to populate
///
/// \throws std::runtime_error If failed to deserialize connectionState
void from_json(const nlohmann::json& j, Connection& msg)
{
  from_json(j, msg.header);

  auto connection_state = j.at("connectionState").get<std::string>();
  if (
    connection_state == Connection::CONNECTION_STATE_ONLINE ||
    connection_state == Connection::CONNECTION_STATE_OFFLINE ||
    connection_state == Connection::CONNECTION_STATE_CONNECTIONBROKEN)
  {
    msg.connection_state = connection_state;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected connectionState.");
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__CONNECTION_HPP_
