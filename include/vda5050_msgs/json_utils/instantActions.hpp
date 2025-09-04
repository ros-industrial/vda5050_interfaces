#include <nlohmann/json.hpp>

#include "vda5050_msgs/json_utils/header.hpp"

#ifndef VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_
#define VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_

namespace vda5050_msgs {

namespace msg {

    void to_json(nlohmann::json& j, const InstantAction& msg)
    {
        to_json(j, msg.header);
        j["actions"] = msg.actions;
    }

} // namespace msg
} // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__INSTANT_ACTIONS_HPP_