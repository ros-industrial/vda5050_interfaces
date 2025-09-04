#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"

#ifndef VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_
#define VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_

namespace vda5050_msgs {

namespace msg {

    /// \brief Convert a vda5050_msgs::msg::ActionParameterValue object to a nlohmann::json object
    /// \param j 
    /// \param msg 
    void to_json(nlohmann::json& j, const ActionParameterValue& msg)
    {
        if (
            msg.type == ActionParameterValue::ARRAY ||
            msg.type == ActionParameterValue::BOOL ||
            msg.type == ActionParameterValue::NUMBER ||
            msg.type == ActionParameterValue::STRING ||
            msg.type == ActionParameterValue::OBJECT)
        {
            j["type"] = msg.type;
        }
        else
        {
            throw std::runtime_error("Serialization error: Unexpected type");
        }
        
        j["value"] = msg.value;
    }

    /// \brief Populate a vda5050_msgs::msg::ActionParameterValue object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized parameter value data
    /// \param msg Reference to the ActionParameterValue message to populate    
    ///
    /// \throws std::runtime_error If failed to deserialize type 
    void from_json(nlohmann::json& j, const ActionParameterValue& msg)
    {
        auto type = j.at("type").get<uint8_t>();
        if (
            type == ActionParameterValue::ARRAY ||
            type == ActionParameterValue::BOOL ||
            type == ActionParameterValue::NUMBER ||
            type == ActionParameterValue::STRING ||
            type == ActionParameterValue::OBJECT)
        {
            msg.type = type;
        }
        else
        {
            throw std::runtime_error("JSON parsing error: Unexpected type.");
        }
    }


    /// @brief 
    /// @param j 
    /// @param msg 
    void to_json(nlohmann::json& j, const ActionParameter& msg)
    {
        j["key"] = msg.key;
        j["value"] = msg.value;
    }

    /// @brief 
    /// @param j 
    /// @param msg 
    void from_json(nlohmann::json& j, const ActionParameter& msg)
    {
        auto key = j.at("key").get<std::string>();
        auto value = j.at("value").get<>(ActionParameterValue);
    }

} // namespace msg
} // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_