#include "follow_ball/control_node.hpp"
#include <cmath>

namespace follow_ball
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/* Constructor takes parameters
Create publisher, subscribers and timer */
FollowerNode::FollowerNode()
: Node("follower_node")
{
  declare_parameter("min_linear", -0.2);
  declare_parameter("max_linear", 0.2);
  declare_parameter("min_angular", -0.7);
  declare_parameter("max_angular", 0.7);

  get_parameter("min_linear", min_linear_);
  get_parameter("max_linear", max_linear_);
  get_parameter("min_angular", min_angular_);
  get_parameter("max_angular", max_angular_);

  laser_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "obstacle", 10, std::bind(&FollowerNode::obstacle_callback, this, _1));
  camera_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "ball", 10, std::bind(&FollowerNode::ball_callback, this, _1));
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "velocity", 10);
  vector1_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "vector1", 10);
  vector2_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "vector2", 10);

  state_ = FOLLOWING_;
  last_ball_side_ = RIGHT_;

  publisher_timer_ = create_wall_timer(
    300ms, std::bind(
      &FollowerNode::state_machine, this));

  RCLCPP_INFO(
    get_logger(),
    "Min and max linear are = %f, %f\nAngulars are: %f, %f\n",
    min_linear_, max_linear_, min_angular_, max_angular_);
}

/* When laser publish repulsive vector, save it */
void
FollowerNode::obstacle_callback(
  const geometry_msgs::msg::Vector3::SharedPtr obstacle_vector)
{
  repulsive_.x = obstacle_vector->x;
  repulsive_.y = obstacle_vector->y;

  // RCLCPP_INFO(
  //   get_logger(),
  //   "Repulsive vector is = x: %f, y:%f\n",
  //   repulsive_.x, repulsive_.y);
}

/* When camera publish its atractive vector,
save it and save in which side it is */
void
FollowerNode::ball_callback(
  const geometry_msgs::msg::Vector3::SharedPtr ball_vector)
{
  atractive_.x = ball_vector->x;
  atractive_.y = ball_vector->y;

  if (atractive_.y > 0) {
    last_ball_side_ = LEFT_;

  } else if (atractive_.y < 0) {
    last_ball_side_ = RIGHT_;
  }

  // RCLCPP_INFO(
  //   get_logger(),
  //   "Atractive vector is = x: %f, y:%f\n",
  //   atractive_.x, atractive_.y);
}

/* When following, calculate resultant vector with
atractive and repulsive ones and publish the
adecuate twist message */
void
FollowerNode::following_callback()
{
  geometry_msgs::msg::Vector3 final_vector;
  visualization_msgs::msg::Marker visual_repulsive;
  visualization_msgs::msg::Marker visual_atractive;

  visual_repulsive = create_visual_vector(repulsive_, 1.0, 0.0, 0.0);
  visual_atractive = create_visual_vector(atractive_, 0.0, 1.0, 0.0);
  final_vector.x = repulsive_.x + atractive_.x;
  final_vector.y = repulsive_.y + atractive_.y;

  RCLCPP_INFO(
    get_logger(),
    "Total vector is = x: %f, y:%f\n",
    final_vector.x, final_vector.y);

  velocity_ = create_twist(final_vector);
  vel_pub_->publish(velocity_);
  vector1_pub_->publish(visual_repulsive);
  vector2_pub_->publish(visual_atractive);
}

/* Create a visual marker vector using a numeric one
Sets its color depending on its parameters*/
visualization_msgs::msg::Marker
FollowerNode::create_visual_vector(
  geometry_msgs::msg::Vector3 & vector, float r, float g, float b)
{
  visualization_msgs::msg::Marker visual_vector;

  visual_vector.header.frame_id = "base_footprint";
  visual_vector.type = visualization_msgs::msg::Marker::LINE_STRIP;
  visual_vector.action = visualization_msgs::msg::Marker::ADD;
  visual_vector.scale.x = 0.1;
  visual_vector.color.a = 1.0;
  visual_vector.color.r = r;
  visual_vector.color.g = g;
  visual_vector.color.b = b;

  geometry_msgs::msg::Point init_point;
  geometry_msgs::msg::Point end_point;
  init_point.x = 0; init_point.y = 0; init_point.z = 0;
  end_point.x = vector.x; end_point.y = vector.y; end_point.z = vector.z;
  visual_vector.points.push_back(init_point);
  visual_vector.points.push_back(end_point);

  return visual_vector;
}

/* When checking, turn left or right
depending on the last side the ball was seen*/
void
FollowerNode::checking_callback()
{
  RCLCPP_INFO(
    get_logger(),
    "Searching for ball...");

  velocity_.linear.x = 0;
  if (last_ball_side_ == LEFT_) {
    velocity_.angular.z = -max_angular_ * 1.2;
  } else {
    velocity_.angular.z = max_angular_ * 1.2;
  }

  vel_pub_->publish(velocity_);
}

/* Create a twist message using a vector
linear vel is = x and angular is = tan(y/x) */
geometry_msgs::msg::Twist
FollowerNode::create_twist(
  geometry_msgs::msg::Vector3 & vector)
{
  geometry_msgs::msg::Twist twist;
  float linear_vel = vector.x;

  // Calcular el error: distancia de la pelota al centro
  float error = vector.y;  // La desviación de la pelota en el eje Y

  // PD Control
  float derivative = error - error_prev_;     // Cambio del error
  float angular_vel = (kp_ * error) + (kd_ * derivative);

  // Guardar el error actual para la próxima iteración
  error_prev_ = error;

  // Limitar velocidades
  twist.linear.x = std::clamp(linear_vel, min_linear_, max_linear_);
  twist.angular.z = std::clamp(-angular_vel, min_angular_, max_angular_);

  return twist;
}

/* State machine of 2 states
When one stops the other begins*/
void
FollowerNode::state_machine()
{
  switch (state_) {

    case FOLLOWING_:
      following_callback();

      if (!check_following()) {
        state_ = CHECKING_;
      }
      break;

    case CHECKING_:
      checking_callback();

      if (check_following()) {
        state_ = FOLLOWING_;
      }
  }
}

/* If atractive vector is not null,
keep following*/
bool
FollowerNode::check_following()
{
  return !(atractive_.x == 0 && atractive_.y == 0);
}

}  // namespace follow_ball
