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

#ifndef TEST__GENERATOR__GENERATOR_HPP_
#define TEST__GENERATOR__GENERATOR_HPP_

#include <limits>
#include <random>
#include <string>
#include <vector>

#include "vda5050_msgs/msg/connection.hpp"
#include "vda5050_msgs/msg/header.hpp"
#include "vda5050_msgs/msg/instant_actions.hpp"
#include "vda5050_msgs/msg/action.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include "vda5050_msgs/msg/action_parameter_value.hpp"

using vda5050_msgs::msg::Connection;
using vda5050_msgs::msg::Header;
using vda5050_msgs::msg::InstantActions;
using vda5050_msgs::msg::Action;
using vda5050_msgs::msg::ActionParameter;
using vda5050_msgs::msg::ActionParameterValue;

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
  std::string generate_connection_state()
  {
    std::vector<std::string> states = {
      Connection::ONLINE, Connection::OFFLINE, Connection::CONNECTIONBROKEN};
    auto state_idx = connection_state_dist_(rng_);
    return states[state_idx];
  }

  /// \brief Generate a random index for enum selection
  uint8_t generate_random_index(size_t size)
  {
    std::uniform_int_distribution<uint8_t> index_dist(0, size - 1);
    return index_dist(rng_);
  }

  /// \brief Generate a random vector of type T
  template <typename T>
  std::vector<T> generate_random_vector(const uint8_t size)
  {
    std::vector<T> vec(size);
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
      *it = generate<T>();
    }
    return vec;
  }

  /// \brief
  uint8_t generate_random_size()
  {
    return size_dist_(rng_);
  }

  /// \brief Generte a random blocking type value
  std::string generate_random_blocking_type()
  {
    std::vector<std::string> states = {Action::NONE, Action::SOFT, Action::HARD};

    auto state_idx = generate_random_index(states.size());

    return states[state_idx];
  }

  /// TODO: @shawnkchan KIV to rename this function. Made it more verbose to be clear
  /// \brief Generate a random ActionParameterValue type
  uint8_t generate_random_action_parameter_value_type()
  {
    std::vector<uint8_t> states = {ActionParameterValue::ARRAY, ActionParameterValue::BOOL, ActionParameterValue::NUMBER, ActionParameterValue::STRING, ActionParameterValue::OBJECT};

    auto state_idx = generate_random_index(states.size());

    return states[state_idx];
  }

  /// \brief Generate a fully populated message of a supported type
  template <typename T>
  T generate()
  {
    if constexpr (std::is_same_v<T, Header>)
    {
      Header msg;
      msg.header_id = generate_uint();
      msg.timestamp = generate_milliseconds();
      msg.version = "2.0.0";  // Fix the VDA 5050 version to 2.0.0
      msg.manufacturer = generate_random_string();
      msg.serial_number = generate_random_string();
      return msg;
    }
    else if constexpr (std::is_same_v<T, Connection>)
    {
      Connection msg;
      msg.header = generate<Header>();
      msg.connection_state = generate_connection_state();
      return msg;
    }
    else if constexpr (std::is_same_v<T, ActionParameterValue>)
    {
      ActionParameterValue msg;
      msg.type = generate_random_action_parameter_value_type();
      msg.value = generate_random_string();
      return msg; 
    }
    else if constexpr (std::is_same_v<T, ActionParameter>)
    {
      ActionParameter msg;
      msg.key = generate_random_string();
      msg.value = generate<ActionParameterValue>();
      return msg;
    }
    else if constexpr (std::is_same_v<T, Action>)
    {
      Action msg;
      msg.action_type = generate_random_string();
      msg.action_id = generate_random_string();
      msg.blocking_type = generate_random_blocking_type();
      msg.action_description.push_back(generate_random_string());
      msg.action_parameters = generate_random_vector<ActionParameter>(generate_random_size());
      return msg;
    }
    else if constexpr (std::is_same_v<T, InstantActions>)
    {
      InstantActions msg;
      msg.header = generate<Header>();
      msg.actions = generate_random_vector<Action>(generate_random_size());
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

  /// \brief Distribution for random vector size 
  std::uniform_int_distribution<uint8_t> size_dist_;
};

#endif  // TEST__GENERATOR__GENERATOR_HPP_
