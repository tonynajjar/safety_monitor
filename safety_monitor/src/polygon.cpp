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

#include "nav2_collision_monitor/polygon.hpp"

#include <exception>
#include <utility>
#include "earcut.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Polygon::Polygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name)
: node_(node), polygon_name_(polygon_name)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Polygon", polygon_name_.c_str());
}

Polygon::~Polygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Polygon", polygon_name_.c_str());
  poly_.clear();
}

bool Polygon::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getParameters()) {
    return false;
  }

  return true;
}

void Polygon::activate()
{
}

void Polygon::deactivate()
{
}

std::string Polygon::getName() const
{
  return polygon_name_;
}

int Polygon::getMaxPoints() const
{
  return max_points_;
}

void Polygon::getPolygon(std::vector<Point> & poly) const
{
  poly = poly_;
}

int Polygon::getPointsInside(const std::vector<Point> & points) const
{
  int num = 0;
  for (const Point & point : points) {
    if (isPointInside(point)) {
      num++;
    }
  }
  return num;
}

visualization_msgs::msg::Marker Polygon::getMarker(bool triggered) const
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  //polygon.points.push_back(polygon.points.front()); // to fill up the polygon
  std::vector<Point> poly;
  getPolygon(poly);
  std::vector<std::vector<std::array<double, 2>>> polygon;
  std::vector<std::array<double, 2>> polylines;

  for(auto & point : poly){
    std::array<double, 2> arr{point.x, point.y};
    polylines.push_back(arr);
  }

  polygon.push_back(polylines);

  std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(polygon);

  visualization_msgs::msg::Marker marker;
  std_msgs::msg::ColorRGBA color;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = node->now();
  marker.ns = polygon_name_;
  marker.id = 0;
  marker.type = marker.TRIANGLE_LIST;
  marker.action = marker.ADD;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = triggered ? 0.8 : 0.1;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  marker.points.reserve(indices.size());
  marker.colors.reserve(indices.size());

  for (std::size_t i = 0; i < indices.size() - 1; i = i+3)
  {
    marker.points.push_back(createPoint(polylines[indices[i]]));
    marker.points.push_back(createPoint(polylines[indices[i+1]]));
    marker.points.push_back(createPoint(polylines[indices[i+2]]));
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 1.0;
    marker.colors.push_back(color);
  }

  return marker;
}


geometry_msgs::msg::Point Polygon::createPoint(const std::array<double, 2>& arr) const
{
  geometry_msgs::msg::Point p;
  p.x = arr[0];
  p.y = arr[1];
  return p;
}

bool Polygon::getCommonParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".frame_id", rclcpp::ParameterValue("scan"));
    frame_id_ = node->get_parameter(polygon_name_ + ".frame_id").as_string();

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".max_points", rclcpp::ParameterValue(3));
    max_points_ = node->get_parameter(polygon_name_ + ".max_points").as_int();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting common polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }
  return true;
}

bool Polygon::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getCommonParameters()) {
    return false;
  }

  try {
    // Leave it not initialized: the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".points", rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> poly_row =
      node->get_parameter(polygon_name_ + ".points").as_double_array();
    // Check for points format correctness
    if (poly_row.size() <= 6 || poly_row.size() % 2 != 0) {
      RCLCPP_ERROR(
        logger_,
        "[%s]: Polygon has incorrect points description",
        polygon_name_.c_str());
      return false;
    }

    // Obtain polygon vertices
    Point point;
    bool first = true;
    for (double val : poly_row) {
      if (first) {
        point.x = val;
      } else {
        point.y = val;
        poly_.push_back(point);
      }
      first = !first;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

inline bool Polygon::isPointInside(const Point & point) const
{
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int poly_size = poly_.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    if ((point.y <= poly_[i].y) == (point.y > poly_[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = poly_[i].x +
        (point.y - poly_[i].y) * (poly_[j].x - poly_[i].x) /
        (poly_[j].y - poly_[i].y);
      // If intersection with checked edge is greater than point.x coordinate, inverting the result
      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

}  // namespace nav2_collision_monitor
