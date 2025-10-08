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

#ifndef VDA5050_MSGS__JSON_UTILS__FACTSHEET_HPP_
#define VDA5050_MSGS__JSON_UTILS__FACTSHEET_HPP_

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/agv_geometry.hpp"
#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/json_utils/load_specification.hpp"
#include "vda5050_msgs/json_utils/physical_parameters.hpp"
#include "vda5050_msgs/json_utils/protocol_features.hpp"
#include "vda5050_msgs/json_utils/protocol_limits.hpp"
#include "vda5050_msgs/json_utils/type_specification.hpp"
#include "vda5050_msgs/json_utils/vehicle_config.hpp"
#include "vda5050_msgs/msg/factsheet.hpp"

namespace vda5050_msgs {

namespace msg {
// ============================================================================
/// \brief  Convert a vda5050_msgs::msg::Factsheet object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Factsheet message object to serialize
void to_json(nlohmann::json& j, const Factsheet& msg)
{
  to_json(j, msg.header);
  j["typeSpecification"] = msg.type_specification;
  j["physicalParameters"] = msg.physical_parameters;
  j["protocolLimits"] = msg.protocol_limits;
  j["protocolFeatures"] = msg.protocol_features;
  j["agvGeometry"] = msg.agv_geometry;
  j["loadSpecification"] = msg.load_specification;
  j["vehicleConfig"] = msg.vehicle_config;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Factsheet object
///
/// \param j Reference to the JSON object containing serialized Factsheet data
/// \param msg Reference to the Factsheet message to populate
void from_json(const nlohmann::json& j, Factsheet& msg)
{
  from_json(j, msg.header);
  msg.type_specification = j.at("typeSpecification").get<TypeSpecification>();
  msg.physical_parameters =
    j.at("physicalParameters").get<PhysicalParameters>();
  msg.protocol_limits = j.at("protocolLimits").get<ProtocolLimits>();
  msg.protocol_features = j.at("protocolFeatures").get<ProtocolFeatures>();
  msg.agv_geometry = j.at("agvGeometry").get<AgvGeometry>();
  msg.load_specification = j.at("loadSpecification").get<LoadSpecification>();
  msg.vehicle_config = j.at("vehicleConfig").get<VehicleConfig>();
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__FACTSHEET_HPP_
