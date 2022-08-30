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

#ifndef NAV2_COLLISION_MONITOR__POLYGON_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "nav2_util/lifecycle_node.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Basic polygon shape class.
 * For STOP/SLOWDOWN model it represents zone around the robot
 * while for APPROACH model it represents robot footprint.
 */
class Polygon
{
public:
  /**
   * @brief Polygon constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of polygon
   */
  Polygon(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name);
  /**
   * @brief Polygon destructor
   */
  virtual ~Polygon();

  /**
   * @brief Shape configuration routine. Obtains ROS-parameters related to shape object
   * and creates polygon lifecycle publisher.
   * @return True in case of everything is configured correctly, or false otherwise
   */
  bool configure();
  /**
   * @brief Activates polygon lifecycle publisher
   */
  void activate();
  /**
   * @brief Deactivates polygon lifecycle publisher
   */
  void deactivate();

  /**
   * @brief Returns the name of polygon
   * @return Polygon name
   */
  std::string getName() const;
  /**
   * @brief Obtains polygon maximum points to enter inside polygon causing no action
   * @return Maximum points to enter to current polygon and take no action
   */
  int getMaxPoints() const;

  /**
   * @brief Gets polygon points
   * @param poly Output polygon points (vertices)
   */
  virtual void getPolygon(std::vector<Point> & poly) const;

  /**
   * @brief Gets number of points inside given polygon
   * @param points Input array of points to be checked
   * @return Number of points inside polygon. If there are no points,
   * returns zero value.
   */
  virtual int getPointsInside(const std::vector<Point> & points) const;

  /**
   * @brief Return visualization marker
   */
  visualization_msgs::msg::Marker getMarker(bool triggered) const;

  /**
   * @brief Helper function for creating a Point
   */
  geometry_msgs::msg::Point createPoint(const double& x, const double& y, const double& z) const;


protected:
  /**
   * @brief Supporting routine obtaining ROS-parameters common for all shapes
   * @return True if all parameters were obtained or false in failure case
   */
  bool getCommonParameters();

  /**
   * @brief Supporting routine obtaining polygon-specific ROS-parameters
   * @return True if all parameters were obtained or false in failure case
   */
  virtual bool getParameters();

  /**
   * @brief Checks if point is inside polygon
   * @param point Given point to check
   * @return True if given point is inside polygon, otherwise false
   */
  bool isPointInside(const Point & point) const;

  // ----- Variables -----

  /// @brief Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  // Basic parameters
  /// @brief Frame name of the source
  std::string frame_id_;
  /// @brief Name of polygon
  std::string polygon_name_;
  /// @brief Maximum number of data readings within a zone to not trigger the action
  int max_points_;
  /// @brief Time step for robot movement simulation
  double simulation_time_step_;

  // Visualization
  /// @brief Polygon points stored for later publishing
  geometry_msgs::msg::Polygon polygon_;

  /// @brief Polygon points (vertices)
  std::vector<Point> poly_;
};  // class Polygon

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_HPP_
