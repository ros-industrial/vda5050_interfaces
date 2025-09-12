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
#include "vda5050_msgs/msg/instant_action.hpp"
#include "vda5050_msgs/msg/action.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include "vda5050_msgs/msg/action_parameter_value.hpp"
#include "vda5050_msgs/msg/order.hpp"

using vda5050_msgs::msg::Connection;
using vda5050_msgs::msg::Header;
using vda5050_msgs::msg::InstantAction;
using vda5050_msgs::msg::Action;
using vda5050_msgs::msg::ActionParameter;
using vda5050_msgs::msg::ActionParameterValue;
using vda5050_msgs::msg::Order;
using vda5050_msgs::msg::ControlPoint;
using vda5050_msgs::msg::Edge;
using vda5050_msgs::msg::Node;
using vda5050_msgs::msg::NodePosition;
using vda5050_msgs::msg::Trajectory;

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

  /// \brief Generate a random 64-bit floating-point number
  double generate_random_float()
  {
    return float_dist_(rng_);
  }

  /// \brief Generate a random boolean value
  bool generate_random_bool()
  {
    return bool_dist_(rng_);
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

  /// \brief Generate a random vector of type float64
  std::vector<double> generate_random_float_vector(const uint8_t size)
  {
    std::vector<double> vec(size);
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
      *it = generate_random_float();
    }
    return vec;
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

  /// \brief Generate a random orientation type value 
  std::string generate_random_orientation_type()
  {
    std::vector<std::string> states = {Edge::TANGENTIAL, Edge::GLOBAL};

    auto state_idx = generate_random_index(states.size());

    return states[state_idx];
  }

  /// \brief Generate a fully populated message of a supported type
  template <typename T>
  T generate()
  {
    T msg;
    if constexpr (std::is_same_v<T, Header>)
    {
      msg.header_id = generate_uint();
      msg.timestamp = generate_milliseconds();
      msg.version = "2.0.0";  // Fix the VDA 5050 version to 2.0.0
      msg.manufacturer = generate_random_string();
      msg.serial_number = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Connection>)
    {
      msg.header = generate<Header>();
      msg.connection_state = generate_connection_state();
    }
    else if constexpr (std::is_same_v<T, ActionParameterValue>)
    {
      msg.type = generate_random_action_parameter_value_type();
      msg.value = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, ActionParameter>)
    {
      msg.key = generate_random_string();
      msg.value = generate<ActionParameterValue>();
    }
    else if constexpr (std::is_same_v<T, Action>)
    {
      msg.action_type = generate_random_string();
      msg.action_id = generate_random_string();
      msg.blocking_type = generate_random_blocking_type();
      msg.action_description.push_back(generate_random_string());
      msg.action_parameters = generate_random_vector<ActionParameter>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, InstantAction>)
    {
      msg.header = generate<Header>();
      msg.actions = generate_random_vector<Action>(generate_random_size());
    }

    else if constexpr (std::is_same_v<T, ControlPoint>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.weight = 1.0; /// TODO (@shawnkchan): Should this be randomized?
    }

    else if constexpr (std::is_same_v<T, NodePosition>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.map_id = generate_random_string();
      msg.theta.push_back(generate_random_float());
      msg.allowed_deviation_xy.push_back(generate_random_float());
      msg.allowed_deviation_theta.push_back(generate_random_float());
      msg.map_description.push_back(generate_random_string());
    }

    else if constexpr (std::is_same_v<T, Node>)
    {
      msg.node_id = generate_random_string();
      msg.sequence_id = generate_uint();
      msg.released = generate_random_bool();
      msg.actions = generate_random_vector<Action>(generate_random_size());
      msg.node_position.push_back(generate<NodePosition>());
      msg.node_description.push_back(generate_random_string());
     
    }

    else if constexpr (std::is_same_v<T, Trajectory>)
    {
      msg.knot_vector = generate_random_float_vector(generate_random_size());
      msg.control_points = generate_random_vector<ControlPoint>(generate_random_size());
      msg.degree = 1.0; /// TODO (@shawnkchan): Should this be randomized?
    }

    else if constexpr (std::is_same_v<T, Edge>)
    {
      msg.edge_id = generate_random_string();
      msg.sequence_id = generate_uint();
      msg.start_node_id = generate_random_string();
      msg.end_node_id = generate_random_string();
      msg.released = generate_random_bool();
      msg.actions = generate_random_vector<Action>(generate_random_size());
      msg.edge_description.push_back(generate_random_string());
      msg.max_speed.push_back(generate_random_float());
      msg.max_height.push_back(generate_random_float());
      msg.min_height.push_back(generate_random_float());
      msg.orientation.push_back(generate_random_float());
      msg.orientation_type = generate_random_orientation_type();
      msg.direction.push_back(generate_random_string());
      msg.rotation_allowed.push_back(generate_random_bool());
      msg.max_rotation_speed.push_back(generate_random_float());
      msg.trajectory.push_back(generate<Trajectory>());
      msg.length.push_back(generate_random_float());
    }

    else if constexpr (std::is_same_v<T, Order>)
    {
      msg.header = generate<Header>();
      msg.order_id = generate_random_string();
      msg.order_update_id = generate_uint();
      msg.nodes = generate_random_vector<Node>(generate_random_size());
      msg.edges = generate_random_vector<Edge>(generate_random_size());
      msg.zone_set_id.push_back(generate_random_string());
    }

    else
    {
      throw std::runtime_error(
        "No random data generator defined for this custom type: " +
        std::string(typeid(T).name()));
    }
    return msg;
  }

private:
  /// \brief Mersenne Twister random number engine
  std::mt19937 rng_;

  /// \brief Distribution for unsigned 32-bit integers
  std::uniform_int_distribution<uint32_t> uint_dist_;

  /// \brief Distribution for 64-bit floating-point numbers
  std::uniform_real_distribution<double> float_dist_;

  /// \brief Distribution for a boolean value
  /// TODO (@shawnkchan): KIV  should we be bounding this between 0 and 1?
  std::uniform_int_distribution<int> bool_dist_{0, 1};

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
