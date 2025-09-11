#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/msg/order.hpp"

#ifndef VDA5050_MSGS__JSON_UTILS__ORDER_HPP_
#define VDA5050_MSGS__JSON_UTILS__ORDER_HPP_

namespace vda5050_msgs {

namespace msg {

    /// @brief control point
    /// @param j 
    /// @param msg 
    void to_json(nlohmann::json& j, const ControlPoint& msg)
    {
        j['x'] = msg.x;
        j['y'] = msg.y;

    
    }

    /// @brief 
    /// @param j 
    /// @param msg 
    void from_json(const nlohmann::json& j, ControlPoint& msg)
    {

    }

    /// @brief edge
    /// @param j 
    /// @param msg 
    void to_json(nlohmann::json& j, const Edge& msg)
    {
        
    }

    
    void from_json(const nlohmann::json& j, Edge& msg)
    {
        
    }

    /// @brief NodePosition
    /// @param j 
    /// @param msg 
    void to_json(nlohmann::json& j, const NodePosition& msg)
    {
        
    }
    
    /// @brief 
    /// @param j 
    /// @param msg 
    void from_json(const nlohmann::json& j, NodePosition& msg)
    {
        
    }
    void to_json(nlohmann::json& j, const ControlPoint& msg)
    {
        
    }
    
    
    /// @brief Node
    /// @param j 
    /// @param msg 
    void from_json(const nlohmann::json& j, Node& msg)
    {
        
    }
    
    
    void to_json(nlohmann::json& j, const Node& msg)
    {
        
    }
    
    /// @brief Trajectory
    /// @param j 
    /// @param msg 
    void from_json(const nlohmann::json& j, Trajectory& msg)
    {
        
    }
    void to_json(nlohmann::json& j, const Trajectory& msg)
    {
        
    }
    
    
    /// @brief Order
    /// @param j 
    /// @param msg 
    void from_json(const nlohmann::json& j, Order& msg)
    {
        
    }

    void to_json(const nlohmann::json& j, Order& msg)
    {
        
    }
}
}

#endif