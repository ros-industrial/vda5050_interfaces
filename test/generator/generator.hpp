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

#include <random>
#include <vda5050_msgs/msg/connection.hpp>
#include <vda5050_msgs/msg/header.hpp>

using namespace vda5050_msgs;

/// \brief Utility class to generate random instances of VDA 5050 message types
class RandomDataGenerator
{
public:
  /// \brief Default constructor using a non-deterministic seed
  RandomDataGenerator()
  : rng_(std::random_device()()),
    uint_dist_(0, std::numeric_limits<uint32_t>::max()),
    string_length_dist_(0, 50),
    milliseconds_dist_(0, 4000000000000L),
    connection_state_dist_(0, 2)
  {
    // Nothing to do
  }

  /// \brief Constructor with a fixed seed for deterministic results
  explicit RandomDataGenerator(uint32_t seed)
  : rng_(seed),
    uint_dist_(0, std::numeric_limits<uint32_t>::max()),
    string_length_dist_(0, 50),
    milliseconds_dist_(0, 4000000000000L),
    connection_state_dist_(0, 3)
  {
    // Nothing to do
  }

  /// \brief Generate a random unsigned 32-bit integer
  uint32_t generate_uint()
  {
    return uint_dist_(rng_);
  }

  /// \brief Generate a random alphanumerical string with length upto 50
  std::string generate_random_string()
  {
    int length = string_length_dist_(rng_);
    std::string s;
    s.reserve(length);

    const std::string charset =
      "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

    std::uniform_int_distribution<size_t> char_index_dist(
      0, charset.length() - 1);

    for (int i = 0; i < length; i++)
    {
      s += charset[char_index_dist(rng_)];
    }
    return s;
  }

  /// \brief Generate a random millisecond timestamp
  int64_t generate_milliseconds()
  {
    return milliseconds_dist_(rng_);
  }

  /// \brief Generate a random connection state value
  uint8_t generate_connection_state()
  {
    return connection_state_dist_(rng_);
  }

  /// \brief Generate a fully populated message of a supported type
  template <typename T>
  T generate()
  {
    if constexpr (std::is_same_v<T, msg::Header>)
    {
      msg::Header msg;
      msg.header_id = generate_uint();
      msg.timestamp = generate_milliseconds();
      msg.version = "2.0.0";  // Fix the VDA 5050 version to 2.0.0
      msg.manufacturer = generate_random_string();
      msg.serial_number = generate_random_string();
      return msg;
    }
    else if constexpr (std::is_same_v<T, msg::Connection>)
    {
      msg::Connection msg;
      msg.header = generate<msg::Header>();
      msg.connection_state = generate_connection_state();
      return msg;
    }
    else
    {
      throw std::runtime_error(
        "No random data generator defined for this custom type: " +
        std::string(typeid(T).name()));
    }
  }

private:
  /// \brief Mersenne Twister random number engine
  std::mt19937 rng_;

  /// \brief Distribution for unsigned 32-bit integers
  std::uniform_int_distribution<uint32_t> uint_dist_;

  /// \brief Distribution for random string lengths
  std::uniform_int_distribution<int> string_length_dist_;

  /// \brief Distribution for milliseconds from epoch
  std::uniform_int_distribution<int64_t> milliseconds_dist_;

  /// \brief Distribution for VDA 5050 connectionState
  std::uniform_int_distribution<uint8_t> connection_state_dist_;
};

