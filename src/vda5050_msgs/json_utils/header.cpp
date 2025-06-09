/**
 * Copyright (C) 2025 ROS Industrial Consortium Asia Pacific
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

#include "vda5050_msgs/json_utils/header.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std::chrono;

namespace vda5050_msgs {

namespace json_utils {

constexpr const char* ISO8601_FORMAT = "%Y-%m-%dT%H:%M:%S";

//=============================================================================
void to_json(nlohmann::json& j, const msg::Header& msg)
{
  j["headerId"] = msg.header_id;

  system_clock::time_point tp{milliseconds(msg.timestamp)};

  std::time_t time_sec = system_clock::to_time_t(tp);
  auto duration = tp.time_since_epoch();
  auto millisec = duration_cast<milliseconds>(duration).count() % 1000;

  std::ostringstream oss;
  oss << std::put_time(std::gmtime(&time_sec), ISO8601_FORMAT);
  oss << "." << std::setw(3) << std::setfill('0') << millisec << "Z";

  j["timestamp"] = oss.str();

  j["version"] = msg.version;
  j["manufacturer"] = msg.manufacturer;
  j["serialNumber"] = msg.serial_number;
}

//=============================================================================
void from_json(nlohmann::json& j, msg::Header& msg)
{
  msg.header_id = j.at("headerId");

  std::tm t = {};
  char sep;
  int millisec = 0;

  std::string timestamp_str = j.at("timestamp");
  std::istringstream ss(timestamp_str);
  ss >> std::get_time(&t, ISO8601_FORMAT);

  ss >> sep;
  if (sep == '.')
  {
    ss >> millisec;
    if (!ss.eof())
    {
      ss.ignore(std::numeric_limits<std::streamsize>::max(), 'Z');
    }
  }

  // TODO(sauk): Add a check to see if the platform supports timegm
  auto tp = system_clock::from_time_t(timegm(&t));
  auto duration =
    duration_cast<milliseconds>(tp.time_since_epoch()) + milliseconds(millisec);

  msg.timestamp = duration.count();

  msg.version = j.at("version");
  msg.manufacturer = j.at("manufacturer");
  msg.serial_number = j.at("serialNumber");
}

}  // namespace json_utils
}  // namespace vda5050_msgs
