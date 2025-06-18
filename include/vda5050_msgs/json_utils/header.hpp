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

#ifndef VDA5050_MSGS__JSON_UTILS__HEADER_HPP_
#define VDA5050_MSGS__JSON_UTILS__HEADER_HPP_

#include <limits>
#include <nlohmann/json.hpp>
#include <string>

#include "vda5050_msgs/msg/header.hpp"

using namespace std::chrono;

namespace vda5050_msgs {

namespace msg {

constexpr const char* ISO8601_FORMAT = "%Y-%m-%dT%H:%M:%S";

/// \brief Convert a vda5050_msgs::msg::Header object to a
/// nlohmann::json object
///
/// \param j Reference to the JSON object to be populated
/// \param msg Reference to the message object to serialize
void to_json(nlohmann::json& j, const Header& msg)
{
  system_clock::time_point tp{milliseconds(msg.timestamp)};

  std::time_t time_sec = system_clock::to_time_t(tp);
  auto duration = tp.time_since_epoch();
  auto millisec = duration_cast<milliseconds>(duration).count() % 1000;

  std::ostringstream oss;
  oss << std::put_time(std::gmtime(&time_sec), ISO8601_FORMAT);
  oss << "." << std::setw(3) << std::setfill('0') << millisec << "Z";
  if (oss.fail())
  {
    throw std::runtime_error("Failed to format timestamp for serialization.");
  }

  j = nlohmann::json{
    {"headerId", msg.header_id},
    {"timestamp", oss.str()},
    {"version", msg.version},
    {"manufacturer", msg.manufacturer},
    {"serialNumber", msg.serial_number}};
}

/// \brief Populate a vda5050_msgs::msg::Header object from a
/// nlohmann::json object
///
/// \param j Reference to the JSON object containing serialized header data
/// \param msg Reference to the Header message to populate
void from_json(const nlohmann::json& j, Header& msg)
{
  try
  {
    msg.header_id = j.at("headerId").get<uint32_t>();

    // Timestamp deserialization
    std::tm t = {};
    char sep;
    int millisec = 0;

    std::string timestamp_str = j.at("timestamp").get<std::string>();
    std::istringstream ss(timestamp_str);
    ss >> std::get_time(&t, ISO8601_FORMAT);

    ss >> sep;
    if (ss.fail() || sep != '.')
    {
      throw std::runtime_error(
        "JSON parsing error for Header: Unexpected character after seconds in "
        "timestamp.");
    }
    else
    {
      ss >> millisec;
      if (ss.fail())
      {
        throw std::runtime_error(
          "JSON parsing error for Header: Failed to parse milliseconds from "
          "timestamp.");
      }

      if (!ss.eof())
      {
        ss.ignore(std::numeric_limits<std::streamsize>::max(), 'Z');
      }
      else
      {
        throw std::runtime_error(
          "JSON parsing error for Header: Expected 'Z' at the end of timestamp "
          "to indicate UTC.");
      }
    }

    // TODO(sauk): Add a check to see if the platform supports timegm
    auto tp = system_clock::from_time_t(timegm(&t));
    auto duration = duration_cast<milliseconds>(tp.time_since_epoch()) +
                    milliseconds(millisec);

    msg.timestamp = duration.count();

    msg.version = j.at("version").get<std::string>();
    msg.manufacturer = j.at("manufacturer").get<std::string>();
    msg.serial_number = j.at("serialNumber").get<std::string>();
  }
  catch (const nlohmann::json::exception& e)
  {
    throw std::runtime_error(
      "JSON parsing error for Header: " + std::string(e.what()));
  }
}

}  // namespace msg
}  // namespace vda5050_msgs

#endif  // VDA5050_MSGS__JSON_UTILS__HEADER_HPP_
