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

#ifndef VDA5050_MSGS__JSON_UTILS__MAX_ARRAY_LENS_HPP_
#define VDA5050_MSGS__JSON_UTILS__MAX_ARRAY_LENS_HPP_

#include <nlohmann/json.hpp>
#include <string>

#include "vda5050_msgs/msg/max_array_lens.hpp"

namespace vda5050_msgs {

namespace msg {

// ============================================================================
/// \brief Convert a vda5050_msgs::msg::MaxArrayLens object to a nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the MaxArrayLens message object to serialize
void to_json(nlohmann::json& j, const MaxArrayLens& msg)
{
  j["orderNodes"] = msg.order_nodes;
  j["orderEdges"] = msg.order_edges;
  j["nodeActions"] = msg.node_actions;
  j["edgeActions"] = msg.edge_actions;
  j["actionsActionsParameters"] = msg.actions_actions_parameters;
  j["instantActions"] = msg.instant_actions;
  j["trajectoryKnotVector"] = msg.trajectory_knot_vector;
  j["trajectoryControlPoints"] = msg.trajectory_control_points;
  j["stateNodeStates"] = msg.state_node_states;
  j["stateEdgeStates"] = msg.state_edge_states;
  j["stateLoads"] = msg.state_loads;
  j["stateActionStates"] = msg.state_action_states;
  j["stateErrors"] = msg.state_errors;
  j["stateInformation"] = msg.state_information;
  j["errorErrorReferences"] = msg.error_error_references;
  j["informationInfoReferences"] = msg.information_info_references;
}

// ============================================================================
/// \brief Convert a nlohmann::json object to a vda5050_msgs::msg::MaxArrayLens object
///
/// \param j Reference to the JSON object containing serialized MaxArrayLens data
/// \param msg Reference to the MaxArrayLens message to populate
void from_json(const nlohmann::json& j, MaxArrayLens& msg)
{
  msg.order_nodes = j.at("orderNodes").get<uint32_t>();
  msg.order_edges = j.at("orderEdges").get<uint32_t>();
  msg.node_actions = j.at("nodeActions").get<uint32_t>();
  msg.edge_actions = j.at("edgeActions").get<uint32_t>();
  msg.actions_actions_parameters =
    j.at("actionsActionsParameters").get<uint32_t>();
  msg.instant_actions = j.at("instantActions").get<uint32_t>();
  msg.trajectory_knot_vector = j.at("trajectoryKnotVector").get<uint32_t>();
  msg.trajectory_control_points =
    j.at("trajectoryControlPoints").get<uint32_t>();
  msg.state_node_states = j.at("stateNodeStates").get<uint32_t>();
  msg.state_edge_states = j.at("stateEdgeStates").get<uint32_t>();
  msg.state_loads = j.at("stateLoads").get<uint32_t>();
  msg.state_action_states = j.at("stateActionStates").get<uint32_t>();
  msg.state_errors = j.at("stateErrors").get<uint32_t>();
  msg.state_information = j.at("stateInformation").get<uint32_t>();
  msg.error_error_references = j.at("errorErrorReferences").get<uint32_t>();
  msg.information_info_references =
    j.at("informationInfoReferences").get<uint32_t>();
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__MAX_ARRAY_LENS_HPP_
