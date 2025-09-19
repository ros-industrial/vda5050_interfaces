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
#include <string>

#include "vda5050_msgs/json_utils/header.hpp"
#include "vda5050_msgs/msg/factsheet.hpp"

namespace vda5050_msgs {

namespace msg {

/// @brief Convert a vda5050_msgs::msg::FactsheetActionParameter object to a nlohmann::json object
///
/// @param j Reference to the JSON object to be populated
/// @param msg Reference to the message object to serialize
///
/// \throws std::runtime_error If failed to serialize value_data_type 
void to_json(nlohmann::json& j, const FactsheetActionParameter& msg)
{
    j["key"] = msg.key;

    if (msg.value_data_type == "BOOL" || msg.value_data_type == "NUMBER" || msg.value_data_type == "INTEGER" || msg.value_data_type == "FLOAT" || msg.value_data_type == "OBJECT" || msg.value_data_type == "ARRAY")
    {
        j["valueDataType"] = msg.value_data_type;
    }
    else
    {
        throw std::runtime_error("Serialization error: Unexpected value_data_type in factsheet_action_parameter field");
    }

    if (!msg.description.empty())
    {
        j["description"] = msg.description;
    }

    if (!msg.is_optional.empty())
    {
        j["isOptional"] = msg.is_optional;
    }
}

/// @brief Convert a nlohmann::json object to a vda5050_msgs::msg::FactsheetActionParameter object
/// @param j Reference to the JSON object containing serialized FactsheetActionParameter data
/// @param msg Reference to the FactsheetActionParameter message to populate
///
/// \throws std::runtime_error If failed to deserialize value_data_type
void from_json(const nlohmann::json& j, FactsheetActionParameter& msg)
{
    auto key = j.at("key").get<std::string>();
    msg.key = key;

    auto value_data_type = j.at("valueDataType").get<std::string>();
    if (value_data_type == "BOOL" || value_data_type == "NUMBER" || value_data_type == "INTEGER" || value_data_type == "FLOAT" || value_data_type == "OBJECT" || value_data_type == "ARRAY")
    {
        msg.value_data_type = value_data_type;
    }
    else
    {
        throw std::runtime_error("Serialization error: Unexpected value_data_type in factsheet_action_parameter field");
    }

    if (j.contains("description"))
    {
        msg.description.push_back(j.at("description").get<std::string>());
    }

    if (j.contains("isOptional"))
    {
        msg.is_optional.push_back(j.at("isOptional").get<bool>());
    }
}

/// \brief AgvAction
void to_json(nlohmann::json& j, const AgvAction& msg)
{
    j["actionType"] = msg.action_type;

    for (std::string scope : msg.action_scopes)
    {
        if (scope != "INSTANT" || scope != "NODE" || scope != "EDGE")
        {
            throw std::runtime_error("Serialization error: Unexpected scope in action_scopes field");
        }
    }
    j["actionScopes"] = msg.action_scopes;

    if (!msg.factsheet_action_parameters.empty())
    {
        j["factsheetActionParameters"] = msg.factsheet_action_parameters;
    }

    if (!msg.result_description.empty())
    {
        j["resultDescription"] = msg.result_description;
    }

    if (!msg.action_description.empty())
    {
        j["actionDescription"] = msg.action_description;
    }

    if (!msg.blocking_types.empty())
    {
        for (std::string type : msg.blocking_types)
        {
            if (type != "NONE" || type != "SOFT" || type !="HARD")
            {
                throw std::runtime_error("Serialization error: Unexpected type in blocking_types field");
            }
        }

        j["blockingTypes"] = msg.blocking_types;
    }
}

void from_json(const nlohmann::json& j, AgvAction& msg)
{
    auto action_type = j.at("actionType").get<std::string>();
    msg.action_type = action_type;

    auto action_scopes = j.at("actionScopes").get<std::vector<std::string>>();
    for (std::string scope : action_scopes)
    {
        if (scope != "INSTANT" || scope != "NODE" || scope != "EDGE")
        {
            throw std::runtime_error("Serialization error: Unexpected scope in action_scopes field");
        }
    }
    msg.action_scopes = action_scopes;

    if (j.contains("factsheet_action_parameters"))
    {
        msg.factsheet_action_parameters = j.at("factsheet_action_parameters").get<std::vector<FactsheetActionParameter>>();
    }

    if (j.contains("result_description"))
    {
        msg.result_description.push_back(j.at("result_description").get<std::string>());
    }

    if (j.contains("action_description"))
    {
        msg.action_description.push_back(j.at("action_description").get<std::string>());
    }

    if (j.contains("blocking_types"))
    {
        auto blocking_types = j.at("blocking_types").get<std::vector<std::string>>();
        for (std::string type : blocking_types)
        {
            if (type != "NONE" || type != "SOFT" || type !="HARD")
            {
                throw std::runtime_error("Serialization error: Unexpected type in blocking_types field");
            }
        }
        msg.blocking_types = blocking_types;
    }
}

/// Position
void to_json(nlohmann::json& j, const Position& msg)
{
    j["x"] = msg.x;
    j["y"] = msg.y;

    if (!msg.theta.empty())
    {
        j["theta"] = msg.theta;
    }
}

void from_json(const nlohmann::json& j, Position& msg)
{
    msg.x = j.at("x").get<double>();
    msg.y = j.at("y").get<double>();

    if (j.contains("theta"))
    {
        msg.theta.push_back(j.at("theta").get<double>());
    }
}

/// WheelDefinition
void to_json(nlohmann::json& j, const WheelDefinition& msg)
{
    if (msg.type == "DRIVE" || msg.type =="CASTER" || msg.type =="FIXED" || msg.type == "MECANUM")
    {
        j["type"] = msg.type;
    }
    else
    {
        throw std::runtime_error("Serialization error: Unexpected type in wheel_definition field");
    }

    j["isActiveDriven"] = msg.is_active_driven;
    j["isActiveSteered"] = msg.is_active_steered;
    j["position"] = msg.position;
    j["diameter"] = msg.diameter;
    j["width"] = msg.width;
    j["centerDisplacement"] = msg.center_displacement;

    if (!msg.constraints.empty())
    {
        j["constraints"] = msg.constraints;
    }

}

void from_json(const nlohmann::json& j, WheelDefinition& msg)
{
    auto type = j.at("type").get<std::string>();
    if (type == "DRIVE" || type =="CASTER" || type =="FIXED" || type == "MECANUM")
    {
        msg.type = type;
    }
    else
    {
        throw std::runtime_error("Serialization error: Unexpected type in wheel_definition field");
    }

    msg.is_active_driven = j.at("isActiveDriven").get<bool>();
    msg.is_active_steered = j.at("isActiveSteered").get<bool>();
    msg.position = j.at("position").get<Position>();
    msg.diameter = j.at("diameter").get<double>();
    msg.width = j.at("width").get<double>();
    msg.center_displacement.push_back(j.at("center_displacement").get<double>());

    if (j.contains("constraints"))
    {
        msg.constraints.push_back(j.at("constraints").get<std::string>());
    }
}

/// PolygonPoint
void to_json(nlohmann::json& j, const PolygonPoint& msg)
{
    j["x"] = msg.x;
    j["y"] = msg.y;
}

void from_json(const nlohmann::json& j, PolygonPoint& msg)
{
    msg.x = j.at("x").get<double>();
    msg.y = j.at("y").get<double>();
}

/// Envelope2d
void to_json(nlohmann::json& j, const Envelope2d& msg)
{
    j["set"] = msg.set;
    j["polygonPoints"] = msg.polygon_points;
    
    if (!msg.description.empty())
    {
        j["description"] = msg.description;
    }
}

void from_json(const nlohmann::json& j, Envelope2d& msg)
{
    msg.set = j.at("set").get<std::string>();
    msg.polygon_points = j.at("polygonPoints").get<std::vector<PolygonPoint>>();

    if (j.contains("description"))
    {
        msg.description.push_back(j.at("description").get<std::string>());
    }
}

/// Envelope3d
void to_json(nlohmann::json& j, const Envelope3d& msg)
{
    j["set"] = msg.set;
    j["format"] = msg.format;

    if (!msg.data.empty())
    {
        j["data"] = msg.data;
    }

    if (!msg.url.empty())
    {
        j["url"] = msg.url;
    }

    if (!msg.description.empty())
    {
        j["description"] = msg.description;
    }
}

void from_json(const nlohmann::json& j, Envelope3d& msg)
{
    msg.set = j.at("set").get<std::string>();
    msg.format = j.at("format").get<std::string>();

    if (j.contains("data"))
    {
        msg.data.push_back(j.at("data").get<std::string>());
    }

    if (j.contains("url"))
    {
        msg.url.push_back(j.at("url").get<std::string>());
    }

    if (j.contains("description"))
    {
        msg.description.push_back(j.at("description").get<std::string>());
    }
}

/// AgvGeometry
void to_json(nlohmann::json& j, const AgvGeometry& msg)
{
    if (!msg.wheel_definitions.empty())
    {
        j["wheelDefinitions"] = msg.wheel_definitions;
    }

    if (!msg.envelopes_2d.empty())
    {
        j["envelopes2d"] = msg.envelopes_2d;
    }

    if (!msg.envelopes_3d.empty())
    {
        j["envelopes3d"] = msg.envelopes_3d;
    }
}

void from_json(const nlohmann::json& j, AgvGeometry& msg)
{
    if (j.contains("wheelDefinitions"))
    {
        msg.wheel_definitions = j.at("wheelDefinitions").get<std::vector<WheelDefinition>>();
    }

    if (j.contains("envelopes2d"))
    {
        msg.envelopes_2d = j.at("envelopes2d").get<std::vector<Envelope2d>>();
    }

    if (j.contains("envelopes3d"))
    {
        msg.envelopes_3d = j.at("envelopes3d").get<std::vector<Envelope3d>>();
    }
}

/// BoundingBoxReference
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// LoadDimensions
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// LoadSet
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// LoadSpecification
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// MaxArrayLens
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// MaxStringLens
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// Network
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// OptionalParameters
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// PhysicalParameters
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// ProtocolFeatures
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// ProtocolLimits
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// Timing
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// TypeSpecification
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// VehicleConfig
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// VersionInfo
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// Factsheet
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

} /// namespace msg
} /// namespace vda5050_msgs

#endif

