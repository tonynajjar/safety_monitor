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

#ifndef NAV2_COLLISION_MONITOR__SCAN_HPP_
#define NAV2_COLLISION_MONITOR__SCAN_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "safety_monitor_msgs/msg/field_states.hpp"

#include "nav2_collision_monitor/source.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Implementation for laser scanner source
 */
class Scan : public Source
{
public:
  /**
   * @brief Scan constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of data source
   * @param source_timeout Maximum time interval in which data is considered valid
   */
  Scan(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const rclcpp::Duration & source_timeout);
  /**
   * @brief Scan destructor
   */
  ~Scan();

  /**
   * @brief Data source configuration routine. Obtains ROS-parameters
   * and creates laser scanner subscriber.
   */
  void configure();

  /**
   * @brief Adds latest data from laser scanner to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   */
  void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const;

protected:
  /**
   * @brief Laser scanner data callback
   * @param msg Shared pointer to LaserScan message
   */
  void dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  // ----- Variables -----

  /// @brief Laser scanner data subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr data_sub_;

  /// @brief FieldStates publisher
  rclcpp_lifecycle::LifecyclePublisher<safety_monitor_msgs::msg::FieldStates>::SharedPtr pub_;

  /// @brief Latest data obtained from laser scanner
  sensor_msgs::msg::LaserScan::ConstSharedPtr data_;
};  // class Scan

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SCAN_HPP_
