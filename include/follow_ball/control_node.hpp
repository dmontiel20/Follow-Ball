#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace follow_ball
{

class FollowerNode : public rclcpp::Node
{
public:
  FollowerNode();

private:
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr laser_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr camera_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vector1_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vector2_pub_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;

  void obstacle_callback(const geometry_msgs::msg::Vector3::SharedPtr);
  void ball_callback(const geometry_msgs::msg::Vector3::SharedPtr);
  void following_callback();
  void checking_callback();

  bool check_following();
  void state_machine();
  geometry_msgs::msg::Twist create_twist(geometry_msgs::msg::Vector3 &);
  visualization_msgs::msg::Marker create_visual_vector(
    geometry_msgs::msg::Vector3 &, float, float, float);

  geometry_msgs::msg::Vector3 repulsive_;
  geometry_msgs::msg::Vector3 atractive_;
  geometry_msgs::msg::Twist velocity_;

  static const int FOLLOWING_ = 0;
  static const int CHECKING_ = 1;
  int state_;

  static const int LEFT_ = 0;
  static const int RIGHT_ = 1;
  int last_ball_side_;

  float min_linear_;
  float max_linear_;
  float min_angular_;
  float max_angular_;

  float error_prev_;
  float kp_ = 1.0;
  float kd_ = 0.1;
};

}  // namespace follow_ball

#endif  // CONTROL_NODE_HPP_
