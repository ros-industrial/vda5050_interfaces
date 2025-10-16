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

#ifndef VDA5050_MSGS__JSON_UTILS__ORDER_HPP_
#define VDA5050_MSGS__JSON_UTILS__ORDER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/edge.hpp"
#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/json_utils/node.hpp"
#include "vda5050_msgs/msg/order.hpp"

namespace vda5050_msgs {

namespace msg {
//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Order object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const Order& msg)
{
  to_json(j, msg.header);

  j["orderId"] = msg.order_id;
  j["orderUpdateId"] = msg.order_update_id;
  j["nodes"] = msg.nodes;
  j["edges"] = msg.edges;

  if (!msg.zone_set_id.empty())
  {
    j["zoneSetId"] = msg.zone_set_id.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Order object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, Order& msg)
{
  from_json(j, msg.header);

  msg.order_id = j.at("orderId").get<std::string>();
  msg.order_update_id = j.at("orderUpdateId").get<uint32_t>();
  msg.nodes = j.at("nodes").get<std::vector<Node>>();
  msg.edges = j.at("edges").get<std::vector<Edge>>();

  if (j.contains("zoneSetId"))
  {
    msg.zone_set_id.push_back(j.at("zoneSetId").get<std::string>());
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  /// VDA5050_MSGS__JSON_UTILS__ORDER_HPP_
