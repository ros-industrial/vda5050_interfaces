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

#include "vda5050_msgs/json_utils/connection.hpp"

#include "vda5050_msgs/json_utils/header.hpp"

namespace vda5050_msgs {

namespace json_utils {

//=============================================================================
void to_json(nlohmann::json& j, const msg::Connection& msg)
{
  to_json(j, msg.header);

  std::string connection_state;
  switch (msg.connection_state)
  {
    case msg::Connection::CONNECTION_STATE_ONLINE:
      connection_state = "ONLINE";
      break;
    case msg::Connection::CONNECTION_STATE_OFFLINE:
      connection_state = "OFFLINE";
      break;
    case msg::Connection::CONNECTION_STATE_CONNECTIONBROKEN:
      connection_state = "CONNECTIONBROKEN";
      break;
    default:
      // TODO(sauk): Change this to throw an error
      connection_state = "UNKNOWN";
      break;
  }
  j["connectionState"] = connection_state;
}

//=============================================================================
void from_json(nlohmann::json& j, msg::Connection& msg)
{
  from_json(j, msg.header);

  auto connection_state = j.at("connectionState");
  if (connection_state == "ONLINE")
  {
    msg.connection_state = msg::Connection::CONNECTION_STATE_ONLINE;
  }
  else if (connection_state == "OFFLINE")
  {
    msg.connection_state = msg::Connection::CONNECTION_STATE_OFFLINE;
  }
  else if (connection_state == "CONNECTIONBROKEN")
  {
    msg.connection_state = msg::Connection::CONNECTION_STATE_CONNECTIONBROKEN;
  }
}

}  // namespace json_utils
}  // namespace vda5050_msgs
