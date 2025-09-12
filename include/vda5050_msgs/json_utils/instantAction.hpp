#ifndef VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_
#define VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_

#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/msg/instant_action.hpp"

namespace vda5050_msgs {

namespace msg {

    /// \brief Convert a vda5050_msgs::msg::ActionParameterValue object to a nlohmann::json object
    ///
    /// \param j 
    /// \param msg
    /// 
    /// \throws std::runtime_error If failed to serialize type
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
    void from_json(const nlohmann::json& j, ActionParameterValue& msg)
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

        auto value = j.at("value").get<std::string>();
        msg.value = value;
    }


    /// \brief convert a vda5050_msgs::msg::ActionParameter object to a nlohmann::json object
    ///
    /// \param j Reference to a JSON object to be populated
    /// \param msg Reference to the message object to serialize
    void to_json(nlohmann::json& j, const ActionParameter& msg)
    {
        j["key"] = msg.key;
        j["value"] = msg.value;
    }

    /// \brief populate a vda5050_msgs::msg::ActionParameter object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized ActionParameter data
    /// \param msg Reference to the ActionParameter message to populate
    void from_json(const nlohmann::json& j, ActionParameter& msg)
    {
        auto key = j.at("key").get<std::string>();
        auto value = j.at("value").get<ActionParameterValue>();

        msg.key = key;
        msg.value = value;
    }

    /// \brief convert a vda5050_msgs::msg::Action object to a nlohmann::json object
    ///
    /// \param j Reference to a JSON object to be populated
    /// \param msg Reference to the messge object to serialize
    ///
    /// \throws std::runtime_error If failed to serialize blockingType
    void to_json(nlohmann::json& j, const Action& msg)
    {
        j["actionType"] = msg.action_type;
        j["actionId"] = msg.action_id;

        if (msg.blocking_type == Action::NONE || msg.blocking_type == Action::SOFT || msg.blocking_type == Action::HARD)
        {
            j["blockingType"] = msg.blocking_type;
        }
        else
        {
            throw std::runtime_error("Serialization error: Unexpected blockingType");
        }

        if (!msg.action_description.empty())
        {
            j["actionDescription"] = msg.action_description.front();
        }

        if (!msg.action_parameters.empty())
        {
            j["actionParameters"] = msg.action_parameters;
        }
    }


    /// \brief populate a vda5050_msgs::msg::Action object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized Action data
    /// \param msg Reference to the Action message to populate
    ///
    /// \throws std::runtime_error If failed to deserialize blockingType
    void from_json(const nlohmann::json& j, Action& msg)
    {
        auto action_type = j.at("actionType").get<std::string>();
        auto action_id = j.at("actionId").get<std::string>();

        msg.action_type = action_type;
        msg.action_id = action_id;

        auto blocking_type = j.at("blockingType").get<std::string>();
        if (blocking_type == Action::NONE || blocking_type == Action::SOFT || blocking_type == Action::HARD)
        {
            msg.blocking_type = blocking_type;
        }
        else
        {
            throw std::runtime_error("JSON parsing error: Unexpected blockingType.");
        }

        if (j.contains("actionDescription"))
        {
            auto action_description = j.at("actionDescription").get<std::string>();
            msg.action_description.push_back(action_description);
        }

        if (j.contains("actionParameters"))
        {
            msg.action_parameters = j.at("actionParameters").get<std::vector<ActionParameter>>();
        }
        
    }

    /// \brief convert a vda5050_msgs::msg::InstantAction object to a nlohmann::json object
    ///
    /// \param j Reference to the JSON object to be populated
    /// \param msg Reference to the message object to serialize
    void to_json(nlohmann::json& j, const InstantAction& msg)
    {
        to_json(j, msg.header);

        j["actions"] = msg.actions;
    }

    /// \brief populate a vda5050_msgs::msg::InstantAction object from a nlohmann::json object
    ///
    /// \param j Reference to the JSON object containing serialized InstantAction data
    /// \param msg Reference to the InstantAction object to be populated
    void from_json(const nlohmann::json& j, InstantAction& msg)
    {
        from_json(j, msg.header);
        
        auto actions = j.at("actions").get<std::vector<Action>>();
        msg.actions = actions;
    }

} // namespace msg
} // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_