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

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/msg/factsheet.hpp"

namespace vda5050_msgs {

namespace msg {




















// ============================================================================
/// \brief Convert a vda5050_msgs::msg::PhysicalParameters object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the PhysicalParameters message object to serialize
void to_json(nlohmann::json& j, const PhysicalParameters& msg)
{
  j["speedMin"] = msg.speed_min;
  j["speedMax"] = msg.speed_max;
  j["accelerationMax"] = msg.acceleration_max;
  j["decelerationMax"] = msg.deceleration_max;
  j["heightMin"] = msg.height_min;
  j["heightMax"] = msg.height_max;
  j["width"] = msg.width;
  j["length"] = msg.length;

  if (!msg.angular_speed_min.empty())
  {
    j["angularSpeedMin"] = msg.angular_speed_min.front();
  }

  if (!msg.angular_speed_max.empty())
  {
    j["angularSpeedMax"] = msg.angular_speed_max.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::PhysicalParameters object
///
/// \param j Reference to the JSON object containing serialized PhysicalParameters data
/// \param msg Reference to the PhysicalParameters message to populate
void from_json(const nlohmann::json& j, PhysicalParameters& msg)
{
  msg.speed_min = j.at("speedMin").get<double>();
  msg.speed_max = j.at("speedMax").get<double>();
  msg.acceleration_max = j.at("accelerationMax").get<double>();
  msg.deceleration_max = j.at("decelerationMax").get<double>();
  msg.height_min = j.at("heightMin").get<double>();
  msg.height_max = j.at("heightMax").get<double>();
  msg.width = j.at("width").get<double>();
  msg.length = j.at("length").get<double>();

  if (j.contains("angularSpeedMin"))
  {
    msg.angular_speed_min.push_back(j.at("angularSpeedMin").get<double>());
  }

  if (j.contains("angularSpeedMax"))
  {
    msg.angular_speed_max.push_back(j.at("angularSpeedMax").get<double>());
  }
}

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

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::ProtocolLimits object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the ProtocolLimits message object to serialize
void to_json(nlohmann::json& j, const ProtocolLimits& msg)
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
void from_json(const nlohmann::json& j, ProtocolLimits& msg)
{
  msg.max_string_lens = j.at("maxStringLens").get<MaxStringLens>();
  msg.max_array_lens = j.at("maxArrayLens").get<MaxArrayLens>();
  msg.timing = j.at("timing").get<Timing>();
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::Timing object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the Timing message object to serialize
void to_json(nlohmann::json& j, const Timing& msg)
{
  j["minOrderInterval"] = msg.min_order_interval;
  j["minStateInterval"] = msg.min_state_interval;

  if (!msg.default_state_interval.empty())
  {
    j["defaultStateInterval"] = msg.default_state_interval.front();
  }

  if (!msg.visualization_interval.empty())
  {
    j["visualizationInterval"] = msg.visualization_interval.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::Timing object
///
/// \param j Reference to the JSON object containing serialized Timing data
/// \param msg Reference to the Timing message to populate
void from_json(const nlohmann::json& j, Timing& msg)
{
  msg.min_order_interval = j.at("minOrderInterval").get<double>();
  msg.min_state_interval = j.at("minStateInterval").get<double>();

  if (j.contains("defaultStateInterval"))
  {
    msg.default_state_interval.push_back(
      j.at("defaultStateInterval").get<double>());
  }

  if (j.contains("visualizationInterval"))
  {
    msg.visualization_interval.push_back(
      j.at("visualizationInterval").get<double>());
  }
}

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
    j["agvKInematic"] = msg.agv_kinematic;
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

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::VehicleConfig object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the VehicleConfig message object to serialize
void to_json(nlohmann::json& j, const VehicleConfig& msg)
{
  if (!msg.versions.empty())
  {
    j["versions"] = msg.versions;
  }

  if (!msg.network.empty())
  {
    j["network"] = msg.network.front();
  }
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::VehicleConfig object
///
/// \param j Reference to the JSON object containing serialized VehicleConfig data
/// \param msg Reference to the VehicleConfig message to populate
void from_json(const nlohmann::json& j, VehicleConfig& msg)
{
  if (j.contains("versions"))
  {
    msg.versions = j.at("versions").get<std::vector<VersionInfo>>();
  }

  if (j.contains("network"))
  {
    msg.network.push_back(j.at("network").get<Network>());
  }
}

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::VersionInfo object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the VersionInfo message object to serialize
void to_json(nlohmann::json& j, const VersionInfo& msg)
{
  j["key"] = msg.key;
  j["value"] = msg.value;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::VersionInfo object
///
/// \param j Reference to the JSON object containing serialized VersionInfo data
/// \param msg Reference to the VersionInfo message to populate
void from_json(const nlohmann::json& j, VersionInfo& msg)
{
  msg.key = j.at("key").get<std::string>();
  msg.value = j.at("value").get<std::string>();
}

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
