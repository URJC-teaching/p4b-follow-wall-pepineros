#include <memory>

#include "FollowWall/FollowWall.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace FollowWall
{

using std::placeholders::_1;

FollowWallNode::FollowWallNode()
: Node("follow_wall_node")
{
  declare_parameter("min_distance", min_distance_);
  declare_parameter("wall_distance", wall_distance_);
  get_parameter("min_distance", min_distance_);
  get_parameter("wall_distance", wall_distance_);

  RCLCPP_INFO(get_logger(), "FollowWallNode set to wall distance: %f m", wall_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_filtered", rclcpp::SensorDataQoS().reliable(),
    std::bind(&FollowWallNode::laser_callback, this, _1));
  
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("obstacle", 100);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void FollowWallNode::move_robot(float linear, float angular)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  cmd_vel_pub_->publish(msg);
}

void FollowWallNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  int min_idx = std::min_element(scan->ranges.begin(), scan->ranges.end()) - scan->ranges.begin();
  float distance_min = scan->ranges[min_idx];
  float angle = scan->angle_min + scan->angle_increment * min_idx;
  while (angle > M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }

  auto obstacle_msg = std_msgs::msg::Bool();
  
  RCLCPP_INFO(get_logger(), "Laser at %f meters, angle: %f", distance_min, angle);

  switch (current_state_)
  {
    case State::FOLLOW_WALL:
      if (distance_min < wall_distance_ - 0.1f) {
        // Gira ligeramente hacia la pared
        move_robot(0.2f, 0.2f);
      } else if (distance_min > wall_distance_ + 0.1f) {
        // Gira ligeramente alejándose de la pared
        move_robot(0.2f, -0.2f);
      } else {
        // Mantén la distancia de la pared
        move_robot(0.2f, 0.0f);
      }

      if (distance_min < min_distance_) {
        // Si detectamos un obstáculo, cambiamos el estado
        current_state_ = State::AVOID_OBSTACLE;
      }

      break;

    case State::SEARCH_WALL:
      if (distance_min < 2.0f) {
        // Si detectamos una pared, comenzamos a seguirla
        current_state_ = State::FOLLOW_WALL;
      } else {
        // Si no detectamos pared, movemos el robot hacia adelante
        move_robot(0.2f, 0.0f);
      }
      break;

    case State::AVOID_OBSTACLE:
      if (distance_min < min_distance_) {
        // Si encontramos un obstáculo, esquivamos hacia la izquierda
        move_robot(0.0f, 0.5f);
      } else {
        // Volver a la acción de seguir la pared
        current_state_ = State::FOLLOW_WALL;
      }
      break;
  }

  obstacle_msg.data = distance_min < min_distance_;
  obstacle_pub_->publish(obstacle_msg);
}

}  // namespace FollowWall
