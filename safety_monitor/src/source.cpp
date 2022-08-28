// Copyright (c) 2022 Samsung Research Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_collision_monitor/source.hpp"

#include <exception>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Source::Source(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const rclcpp::Duration & source_timeout)
: node_(node), source_name_(source_name), source_timeout_(source_timeout)
{
}

Source::~Source()
{
}

void Source::getCommonParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".topic",
    rclcpp::ParameterValue("scan"));  // Set default topic for laser scanner
  source_topic = node->get_parameter(source_name_ + ".topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".topic_out",
    rclcpp::ParameterValue("out"));  // Set default topic for laser scanner
  std::string topic_out = node->get_parameter(source_name_ + ".topic_out").as_string();

  pub_ = node->create_publisher<safety_monitor_msgs::msg::FieldStates>(
    topic_out, rclcpp::SystemDefaultsQoS()); // TODO: move to a configuration step
  pub_->on_activate();
}

bool Source::sourceValid(
  const rclcpp::Time & source_time,
  const rclcpp::Time & curr_time) const
{
  // Source is considered as not valid, if latest received data timestamp is earlier
  // than current time by source_timeout_ interval
  const rclcpp::Duration dt = curr_time - source_time;
  if (dt > source_timeout_) {
    RCLCPP_WARN(
      logger_,
      "[%s]: Latest source and current collision monitor node timestamps differ on %f seconds. "
      "Ignoring the source.",
      source_name_.c_str(), dt.seconds());
    return false;
  }

  return true;
}

}  // namespace nav2_collision_monitor
