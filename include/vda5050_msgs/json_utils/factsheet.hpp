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

/// \brief AgvAction
void to_json(nlohmann::json& j, const & msg))
{
    j["actionType"] = msg.action_type;

    for (string scope : msg.action_scopes)
    {
        if (scope != "INSTANT" || scope != "NODE" || scope != "EDGE")
        {
            throw std::runtime_error("Serialization error: Unexpected scope in action_scopes field");
        }
    }
    j["actionScopes"] = msg.action_scopes;

    if (!msg.factsheet_action_parameters.empty)
    {
        j["factsheetActionParameters"]
    }

    if (!msg.result_description)
    {
        
    }

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// AgvGeometry
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// BoundingBoxReference
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// Envelope2d
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// Envelope3d
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// FactsheetActionParameter
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

/// PolygonPoint
void to_json(nlohmann::json& j, const & msg))
{

}

void from_json(const nlohmann::json& j, & msg)
{

}

/// Position
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

/// WheelDefinition
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

