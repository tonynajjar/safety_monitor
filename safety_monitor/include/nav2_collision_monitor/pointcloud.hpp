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

#ifndef NAV2_COLLISION_MONITOR__POINTCLOUD_HPP_
#define NAV2_COLLISION_MONITOR__POINTCLOUD_HPP_

#include "safety_monitor_msgs/msg/field_states.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "nav2_collision_monitor/source.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Implementation for pointcloud source
 */
class PointCloud : public Source
{
public:
  /**
   * @brief PointCloud constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of data source
   * @param source_timeout Maximum time interval in which data is considered valid
   */
  PointCloud(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const rclcpp::Duration & source_timeout);
  /**
   * @brief PointCloud destructor
   */
  ~PointCloud();

  /**
   * @brief Data source configuration routine. Obtains pointcloud related ROS-parameters
   * and creates pointcloud subscriber.
   */
  void configure();

  /**
   * @brief Adds latest data from pointcloud source to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   */
  void getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) const;

protected:
  /**
   * @brief Getting sensor-specific ROS-parameters
   * @param source_topic Output name of source subscription topic
   * @param topic_out Topic name for triggered polygons
   */
  void getParameters(std::string & source_topic, std::string & topic_out);

  /**
   * @brief PointCloud data callback
   * @param msg Shared pointer to PointCloud message
   */
  void dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // ----- Variables -----

  /// @brief PointCloud data subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr data_sub_;

  /// @brief FieldStates publisher
  rclcpp_lifecycle::LifecyclePublisher<safety_monitor_msgs::msg::FieldStates>::SharedPtr pub_;

  // Minimum and maximum height of PointCloud projected to 2D space
  double min_height_, max_height_;

  /// @brief Latest data obtained from pointcloud
  sensor_msgs::msg::PointCloud2::ConstSharedPtr data_;
};  // class PointCloud

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POINTCLOUD_HPP_
