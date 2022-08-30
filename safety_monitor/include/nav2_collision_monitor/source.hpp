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

#ifndef NAV2_COLLISION_MONITOR__SOURCE_HPP_
#define NAV2_COLLISION_MONITOR__SOURCE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "safety_monitor_msgs/msg/field_states.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_util/lifecycle_node.hpp"

#include "nav2_collision_monitor/polygon.hpp"
#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Basic data source class
 */
class Source
{
public:
  /**
   * @brief Source constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of data source
   * @param source_timeout Maximum time interval in which data is considered valid
   */
  Source(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const rclcpp::Duration & source_timeout);
  /**
   * @brief Source destructor
   */
  virtual ~Source();

  /**
   * @brief Adds latest data from source to the data array.
   * Empty virtual method intended to be used in child implementations.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   */
  virtual void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const = 0;

  /**
   * @brief Publish FieldStates and Markers
   */
  void publish(rclcpp::Time curr_time) const;

  /// @brief Polygons array
  std::vector<std::shared_ptr<Polygon>> polygons_;

  /// @brief FieldStates publisher
  rclcpp_lifecycle::LifecyclePublisher<safety_monitor_msgs::msg::FieldStates>::SharedPtr field_states_pub_;

  /// @brief MarkerArray publisher
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

protected:
  /**
   * @brief Supporting routine obtaining ROS-parameters common for all data sources
   * @param source_topic Output name of source subscription topic
   */
  void getCommonParameters(std::string & source_topic);

  /**
   * @brief Checks whether the source data might be considered as valid
   * @param source_time Timestamp of latest obtained data
   * @param curr_time Current node time for source verification
   * @return True if data source is valid, otherwise false
   */
  bool sourceValid(
    const rclcpp::Time & source_time,
    const rclcpp::Time & curr_time) const;

  // ----- Variables -----

  /// @brief Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  // Basic parameters
  /// @brief Name of data source
  std::string source_name_;

  // Global variables
  /// @brief Maximum time interval in which data is considered valid
  rclcpp::Duration source_timeout_;
};  // class Source

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SOURCE_HPP_
