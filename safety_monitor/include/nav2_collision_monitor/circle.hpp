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

#ifndef NAV2_COLLISION_MONITOR__CIRCLE_HPP_
#define NAV2_COLLISION_MONITOR__CIRCLE_HPP_

#include "nav2_collision_monitor/polygon.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Circle shape implementaiton.
 * For STOP/SLOWDOWN model it represents zone around the robot
 * while for APPROACH model it represents robot footprint.
 */
class Circle : public Polygon
{
public:
  /**
   * @brief Circle class constructor
   * @param node Collision Monitor node pointer
   * @param polygon_name Name of circle
   */
  Circle(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name);
  /**
   * @brief Circle class destructor
   */
  ~Circle();

  /**
   * @brief Gets polygon points, approximated to the circle.
   * To be used in visualization purposes.
   * @param poly Output polygon points (vertices)
   */
  void getPolygon(std::vector<Point> & poly) const override;

  /**
   * @brief Gets number of points inside circle
   * @param points Input array of points to be checked
   * @return Number of points inside circle. If there are no points,
   * returns zero value.
   */
  int getPointsInside(const std::vector<Point> & points) const override;

protected:
  /**
   * @brief Supporting routine obtaining polygon-specific ROS-parameters
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters() override;

  // ----- Variables -----

  /// @brief Radius of the circle
  double radius_;
  /// @brief (radius * radius) value. Stored for optimization.
  double radius_squared_;
};  // class Circle

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__CIRCLE_HPP_
