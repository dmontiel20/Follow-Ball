// Copyright 2024 Intelligent Robotics Lab
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

#include <memory>
#include <cmath>

#include "follow_ball/obstacle_detector.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"

namespace follow_ball
{

using std::placeholders::_1;

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node")
{
  declare_parameter("limit_distance", limit_distance_);
  get_parameter("limit_distance", limit_distance_);
  declare_parameter("inversion", inversion_);
  get_parameter("inversion", inversion_);

  RCLCPP_INFO(get_logger(), "ObstacleDetectorNode set to %f m", limit_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS().reliable(),
    std::bind(&ObstacleDetectorNode::laser_callback, this, _1));
  obstacle_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    "obstacle", 100);
}


/* Detecta el objeto más cercano al kobuki
Publica un vector repulsivo al objeto
Cuanto más cerca este el objeto, más grande será el repulsivo */
void
ObstacleDetectorNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  int min_idx = std::min_element(scan->ranges.begin(), scan->ranges.end()) - scan->ranges.begin();
  float distance_min = scan->ranges[min_idx];

  auto rep_vector = geometry_msgs::msg::Vector3();

  if (min_limit_ < distance_min && distance_min < limit_distance_) {
    float angle = scan->angle_min + scan->angle_increment * min_idx;
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}

    RCLCPP_INFO(get_logger(), "Obstacle in = %f and angle = %f\n", distance_min, angle);

    float rep_distance = limit_distance_ - distance_min;
    rep_vector.x = inversion_ * -cosf(angle) * rep_distance;
    rep_vector.y = inversion_ * sinf(angle) * rep_distance;

  } else {
    rep_vector.x = 0;
    rep_vector.y = 0;
  }

  obstacle_pub_->publish(rep_vector);
}

}  // namespace follow_ball
