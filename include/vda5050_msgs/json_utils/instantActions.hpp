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
        to_json(j, msg.header);

        if (
            
        )
    }

} // namespace msg
} // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_