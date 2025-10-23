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

#ifndef VDA5050_INTERFACES__JSON_UTILS__BATTERY_STATE_HPP_
#define VDA5050_INTERFACES__JSON_UTILS__BATTERY_STATE_HPP_

#include <nlohmann/json.hpp>

#include "vda5050_interfaces/msg/battery_state.hpp"

namespace vda5050_interfaces {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_interfaces::msg::BatteryState object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const BatteryState& msg)
{
  j["batteryCharge"] = msg.battery_charge;
  j["charging"] = msg.charging;

  if (!msg.battery_voltage.empty())
  {
    j["batteryVoltage"] = msg.battery_voltage.front();
  }

  if (!msg.battery_health.empty())
  {
    j["batteryHealth"] = msg.battery_health.front();
  }

  if (!msg.reach.empty())
  {
    j["reach"] = msg.reach.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_interfaces::msg::BatteryState object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, BatteryState& msg)
{
  msg.battery_charge = j.at("batteryCharge").get<double>();
  msg.charging = j.at("charging").get<bool>();

  if (j.contains("batteryVoltage"))
  {
    msg.battery_voltage.push_back(j.at("batteryVoltage").get<double>());
  }

  if (j.contains("batteryHealth"))
  {
    msg.battery_health.push_back(j.at("batteryHealth").get<int8_t>());
  }

  if (j.contains("reach"))
  {
    msg.reach.push_back(j.at("reach").get<uint32_t>());
  }
}

}  // namespace msg
}  // namespace vda5050_interfaces

#endif  // VDA5050_INTERFACES__JSON_UTILS__BATTERY_STATE_HPP_
