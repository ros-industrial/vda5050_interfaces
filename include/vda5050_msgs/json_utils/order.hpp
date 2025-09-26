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

#include <string>
#include <vector>
#include <cstdint>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/json_utils/instantAction.hpp"  /// TODO: [@shawnkchan] Requires instantAction header from instantActions branch
#include "vda5050_msgs/msg/order.hpp"

namespace vda5050_msgs {

namespace msg {

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Order object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const ControlPoint& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["weight"] = msg.weight;
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::ControlPoint object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, ControlPoint& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.weight = j.at("weight").get<double>();
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Trajectory object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const Trajectory& msg)
{
  j["knotVector"] = msg.knot_vector;
  j["controlPoints"] = msg.control_points;
  j["degree"] = msg.degree;
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Trajectory object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, Trajectory& msg)
{
  msg.knot_vector = j.at("knotVector").get<std::vector<double>>();
  msg.control_points = j.at("controlPoints").get<std::vector<ControlPoint>>();
  msg.degree = j.at("degree").get<double>();
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Edge object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize orientationType
void to_json(nlohmann::json& j, const Edge& msg)
{
  j["edgeId"] = msg.edge_id;
  j["sequenceId"] = msg.sequence_id;
  j["startNodeId"] = msg.start_node_id;
  j["endNodeId"] = msg.end_node_id;
  j["released"] = msg.released;
  j["actions"] = msg.actions;

  if (!msg.edge_description.empty())
  {
    j["edgeDescription"] = msg.edge_description.front();
  }

  if (!msg.max_speed.empty())
  {
    j["maxSpeed"] = msg.max_speed.front();
  }

  if (!msg.max_height.empty())
  {
    j["maxHeight"] = msg.max_height.front();
  }

  if (!msg.min_height.empty())
  {
    j["minHeight"] = msg.min_height.front();
  }

  if (!msg.orientation.empty())
  {
    j["orientation"] = msg.orientation.front();
  }

  if (
    msg.orientation_type == msg.TANGENTIAL ||
    msg.orientation_type == msg.GLOBAL)
  {
    j["orientationType"] = msg.orientation_type;
  }
  else
  {
    throw std::runtime_error(
      "Serialization error: Unexpected orientationType.");
  }

  if (!msg.direction.empty())
  {
    j["direction"] = msg.direction.front();
  }

  if (!msg.rotation_allowed.empty())
  {
    j["rotationAllowed"] = msg.rotation_allowed.front();
  }

  if (!msg.max_rotation_speed.empty())
  {
    j["maxRotationSpeed"] = msg.max_rotation_speed.front();
  }

  if (!msg.trajectory.empty())
  {
    j["trajectory"] = msg.trajectory.front();
  }

  if (!msg.length.empty())
  {
    j["length"] = msg.length.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Edge object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
///
/// \throws std::runtime_error If failed to deserialize orientationType
void from_json(const nlohmann::json& j, Edge& msg)
{
  msg.edge_id = j.at("edgeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<int32_t>();
  msg.start_node_id = j.at("startNodeId").get<std::string>();
  msg.end_node_id = j.at("endNodeId").get<std::string>();
  msg.released = j.at("released").get<bool>();
  msg.actions = j.at("actions").get<std::vector<Action>>();

  if (j.contains("edgeDescription"))
  {
    msg.edge_description.push_back(j.at("edgeDescription").get<std::string>());
  }

  if (j.contains("maxSpeed"))
  {
    msg.max_speed.push_back(j.at("maxSpeed").get<double>());
  }

  if (j.contains("maxHeight"))
  {
    msg.max_height.push_back(j.at("maxHeight").get<double>());
  }

  if (j.contains("minHeight"))
  {
    msg.min_height.push_back(j.at("minHeight").get<double>());
  }

  if (j.contains("orientation"))
  {
    msg.orientation.push_back(j.at("orientation").get<double>());
  }

  auto orientation_type = j.at("orientationType").get<std::string>();
  if (orientation_type == Edge::TANGENTIAL || orientation_type == Edge::GLOBAL)
  {
    msg.orientation_type = orientation_type;
  }
  else
  {
    throw std::runtime_error("JSON parsing error: Unexpected orientationType.");
  }

  if (j.contains("direction"))
  {
    msg.direction.push_back(j.at("direction").get<std::string>());
  }

  if (j.contains("rotationAllowed"))
  {
    msg.rotation_allowed.push_back(j.at("rotationAllowed").get<bool>());
  }

  if (j.contains("maxRotationSpeed"))
  {
    msg.max_rotation_speed.push_back(j.at("maxRotationSpeed").get<double>());
  }

  if (j.contains("trajectory"))
  {
    msg.trajectory.push_back(j.at("trajectory").get<Trajectory>());
  }

  if (j.contains("length"))
  {
    msg.length.push_back(j.at("length").get<double>());
  }
}

//=============================================================================
/// \brief NodePosition
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const NodePosition& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["mapId"] = msg.map_id;

  if (!msg.theta.empty())
  {
    j["theta"] = msg.theta.front();
  }

  if (!msg.allowed_deviation_xy.empty())
  {
    j["allowedDeviationXY"] = msg.allowed_deviation_xy.front();
  }

  if (!msg.allowed_deviation_theta.empty())
  {
    j["allowedDeviationTheta"] = msg.allowed_deviation_theta.front();
  }

  if (!msg.map_description.empty())
  {
    j["mapDescription"] = msg.map_description.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::NodePosition object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, NodePosition& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.map_id = j.at("mapId").get<std::string>();

  if (j.contains("theta"))
  {
    msg.theta.push_back(j.at("theta").get<double>());
  }

  if (j.contains("allowedDeviationXY"))
  {
    msg.allowed_deviation_xy.push_back(
      j.at("allowedDeviationXY").get<double>());
  }

  if (j.contains("allowedDeviationTheta"))
  {
    msg.allowed_deviation_theta.push_back(
      j.at("allowedDeviationTheta").get<double>());
  }

  if (j.contains("mapDescription"))
  {
    msg.map_description.push_back(j.at("mapDescription").get<std::string>());
  }
}

//=============================================================================
/// \brief Convert a vda5050_msgs::msg::Node object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const Node& msg)
{
  j["nodeId"] = msg.node_id;
  j["sequenceId"] = msg.sequence_id;
  j["released"] = msg.released;
  j["actions"] = msg.actions;

  if (!msg.node_position.empty())
  {
    j["nodePosition"] = msg.node_position.front();
  }

  if (!msg.node_description.empty())
  {
    j["nodeDescription"] = msg.node_description.front();
  }
}

//=============================================================================
/// \brief Populate a vda5050_msgs::msg::Node object from a nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized data
/// \param msg Reference to the message object to populate
void from_json(const nlohmann::json& j, Node& msg)
{
  msg.node_id = j.at("nodeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<uint32_t>();
  msg.released = j.at("released").get<bool>();
  msg.actions = j.at("actions").get<std::vector<Action>>();

  if (j.contains("nodePosition"))
  {
    msg.node_position.push_back(j.at("nodePosition").get<NodePosition>());
  }

  if (j.contains("nodeDescription"))
  {
    msg.node_description.push_back(j.at("nodeDescription").get<std::string>());
  }
}

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
