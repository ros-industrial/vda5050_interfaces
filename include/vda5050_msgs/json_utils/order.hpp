#ifndef VDA5050_MSGS__JSON_UTILS__ORDER_HPP_
#define VDA5050_MSGS__JSON_UTILS__ORDER_HPP_

#include <nlohmann/json.hpp>

#include <string>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/json_utils/instantAction.hpp" /// TODO: [@shawnkchan] Requires instantAction header from instantActions branch
#include "vda5050_msgs/msg/order.hpp"

namespace vda5050_msgs {

namespace msg {

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

    /// \brief Populate a vda5050_msgs::msg::ControlPoint object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized data
    /// \param msg Reference to the message object to populate
    void from_json(const nlohmann::json& j, ControlPoint& msg)
    {
        auto x = j.at("x").get<double>();
        msg.x = x;

        auto y = j.at("y").get<double>();
        msg.y = y;

        auto weight = j.at("weight").get<double>();
        msg.weight = weight;
    }

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

    /// \brief Populate a vda5050_msgs::msg::Trajectory object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized data
    /// \param msg Reference to the message object to populate
    void from_json(const nlohmann::json& j, Trajectory& msg)
    {
        auto knot_vector = j.at("knotVector").get<std::vector<double>>();
        msg.knot_vector = knot_vector;

        auto control_points = j.at("controlPoints").get<std::vector<ControlPoint>>();
        msg.control_points = control_points;

        auto degree = j.at("degree").get<double>();
        msg.degree = degree;
    }

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

        if (msg.orientation_type == msg.TANGENTIAL || msg.orientation_type == msg.GLOBAL)
        {
            j["orientationType"] = msg.orientation_type;
        }
        else
        {
            throw std::runtime_error("JSON parsing error: Unexpected orientationType.");
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

    /// \brief Populate a vda5050_msgs::msg::Edge object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized data
    /// \param msg Reference to the message object to populate
    ///
    /// \throws std::runtime_error If failed to deserialize orientationType
    void from_json(const nlohmann::json& j, Edge& msg)
    {
        auto edge_id = j.at("edgeId").get<std::string>();
        msg.edge_id = edge_id;

        auto sequence_id = j.at("sequenceId").get<int32_t>();
        msg.sequence_id = sequence_id;

        auto start_node_id = j.at("startNodeId").get<std::string>();
        msg.start_node_id = start_node_id;

        auto end_node_id = j.at("endNodeId").get<std::string>();
        msg.end_node_id = end_node_id;

        auto released = j.at("released").get<bool>();
        msg.released = released;

        auto actions = j.at("actions").get<std::vector<Action>>();
        msg.actions = actions;

        if (j.contains("edgeDescription"))
        {
            auto edge_description = j.at("edgeDescription").get<std::string>();
            msg.edge_description.push_back(edge_description);
        }

        if (j.contains("maxSpeed"))
        {
            auto max_speed = j.at("maxSpeed").get<double>();
            msg.max_speed.push_back(max_speed);
        }

        if (j.contains("maxHeight"))
        {
            auto max_height = j.at("maxHeight").get<double>();
            msg.max_height.push_back(max_height);
        }

        if (j.contains("minHeight"))
        {
            auto min_height = j.at("minHeight").get<double>();
            msg.min_height.push_back(min_height);
        }

        if (j.contains("orientation"))
        {
            auto orientation = j.at("orientation").get<double>();
            msg.orientation.push_back(orientation);
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
            auto direction = j.at("direction").get<std::string>();
            msg.direction.push_back(direction);
        }

        if (j.contains("rotationAllowed"))
        {
            auto rotation_allowed = j.at("rotationAllowed").get<bool>();
            msg.rotation_allowed.push_back(rotation_allowed);
        }

        if (j.contains("maxRotationSpeed"))
        {
            auto max_rotation_speed = j.at("maxRotationSpeed").get<double>();
            msg.max_rotation_speed.push_back(max_rotation_speed);
        }

        if (j.contains("trajectory"))
        {
            auto trajectory = j.at("trajectory").get<Trajectory>();
            msg.trajectory.push_back(trajectory);
        }

        if (j.contains("length"))
        {
            auto length = j.at("length").get<double>();
            msg.length.push_back(length);
        }
    }

    /// \brief NodePosition
    ///
    /// \param j 
    /// \param msg 
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
    
    /// \brief Populate a vda5050_msgs::msg::NodePosition object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized data
    /// \param msg Reference to the message object to populate
    void from_json(const nlohmann::json& j, NodePosition& msg)
    {
        auto x = j.at("x").get<double>();
        msg.x = x;

        auto y = j.at("y").get<double>();
        msg.y = y;

        auto map_id = j.at("mapId").get<std::string>();
        msg.map_id = map_id;

        if (j.contains("theta"))
        {
            auto theta = j.at("theta").get<double>();
            msg.theta.push_back(theta);
        }

        if (j.contains("allowedDeviationXY"))
        {
            auto allowed_deviation_xy = j.at("allowedDeviationXY").get<double>();
            msg.allowed_deviation_xy.push_back(allowed_deviation_xy);
        }

        if (j.contains("allowedDeviationTheta"))
        {
            auto allowed_deviation_theta = j.at("allowedDeviationTheta").get<double>();
            msg.allowed_deviation_theta.push_back(allowed_deviation_theta);
        }

        if (j.contains("mapDescription"))
        {
            auto map_description = j.at("mapDescription").get<std::string>();
            msg.map_description.push_back(map_description);
        }
    }

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
    
    /// \brief Populate a vda5050_msgs::msg::Node object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized data
    /// \param msg Reference to the message object to populate
    void from_json(const nlohmann::json& j, Node& msg)
    {
        auto node_id = j.at("nodeId").get<std::string>();
        msg.node_id = node_id;

        auto sequence_id = j.at("sequenceId").get<uint32_t>();
        msg.sequence_id = sequence_id;

        auto released = j.at("released").get<bool>();
        msg.released = released;

        msg.actions = j.at("actions").get<std::vector<Action>>();

        if (j.contains("nodePosition"))
        {
            auto node_position = j.at("nodePosition").get<NodePosition>();
            msg.node_position.push_back(node_position);
        }

        if (j.contains("nodeDescription"))
        {
            auto node_description = j.at("nodeDescription").get<std::string>();
            msg.node_description.push_back(node_description);
        }
    }
    
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

    /// \brief Populate a vda5050_msgs::msg::Order object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized data
    /// \param msg Reference to the message object to populate
    void from_json(const nlohmann::json& j, Order& msg)
    {
        from_json(j, msg.header);

        auto order_id = j.at("orderId").get<std::string>();
        msg.order_id = order_id;

        auto order_update_id = j.at("orderUpdateId").get<uint32_t>();
        msg.order_update_id = order_update_id;

        msg.nodes = j.at("nodes").get<std::vector<Node>>();
        
        msg.edges = j.at("edges").get<std::vector<Edge>>();

        if (j.contains("zoneSetId"))
        {
            auto zone_set_id = j.at("zoneSetId").get<std::string>();
            msg.zone_set_id.push_back(zone_set_id);
        }
    }
} /// namespace msg
} /// namespace vda5050_msgs

#endif /// VDA5050_MSGS__JSON_UTILS__ORDER_HPP_

