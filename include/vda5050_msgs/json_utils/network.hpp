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

#ifndef VDA5050_MSGS__JSON_UTILS__NETWORK_HPP_
#define VDA5050_MSGS__JSON_UTILS__NETWORK_HPP_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "vda5050_msgs/msg/network.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Network object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Network message object to serialize
void to_json(nlohmann::json& j, const Network& msg)
{
  if (!msg.dns_servers.empty())
  {
    j["dnsServers"] = msg.dns_servers;
  }

  if (!msg.ntp_servers.empty())
  {
    j["ntpServers"] = msg.ntp_servers;
  }

  if (!msg.local_ip_address.empty())
  {
    j["localIpAddress"] = msg.local_ip_address.front();
  }

  if (!msg.netmask.empty())
  {
    j["netmask"] = msg.netmask.front();
  }

  if (!msg.default_gateway.empty())
  {
    j["defaultGateway"] = msg.default_gateway.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Network object
///
/// \param j Reference to the JSON object containing serialized Network data
/// \param msg Reference to the Network message to populate
void from_json(const nlohmann::json& j, Network& msg)
{
  if (j.contains("dnsServers"))
  {
    msg.dns_servers = j.at("dnsServers").get<std::vector<std::string>>();
  }

  if (j.contains("ntpServers"))
  {
    msg.ntp_servers = j.at("ntpServers").get<std::vector<std::string>>();
  }

  if (j.contains("localIpAddress"))
  {
    msg.local_ip_address.push_back(j.at("localIpAddress").get<std::string>());
  }

  if (j.contains("netmask"))
  {
    msg.netmask.push_back(j.at("netmask").get<std::string>());
  }

  if (j.contains("defaultGateway"))
  {
    msg.default_gateway.push_back(j.at("defaultGateway").get<std::string>());
  }
}

} // namespace msg
} // namespace vda5050_msgs

#endif // VDA5050_MSGS__JSON_UTILS__NETWORK_HPP_
