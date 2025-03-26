#include <memory>
#include <limits>
#include "FollowWall/FollowWall.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace FollowWall
{

using std::placeholders::_1;

FollowWallNode::FollowWallNode()
: Node("follow_wall_node"), current_state_(State::SEARCH_WALL)
{
  declare_parameter("min_distance", min_distance_);
  declare_parameter("wall_distance", wall_distance_);
  get_parameter("min_distance", min_distance_);
  get_parameter("wall_distance", wall_distance_);

  RCLCPP_INFO(get_logger(), "FollowWallNode set to wall distance: %f m", wall_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_raw", rclcpp::SensorDataQoS().reliable(),
    std::bind(&FollowWallNode::laser_callback, this, _1));
  
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("obstacle", 10);
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
  float distance_min = std::numeric_limits<float>::infinity();
  int min_idx = -1;

  // Filtrar valores inválidos (NaN, inf) y encontrar la menor distancia válida
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    if (std::isfinite(scan->ranges[i]) && scan->ranges[i] > 0.05) { // Evita valores inválidos
      if (scan->ranges[i] < distance_min) {
        distance_min = scan->ranges[i];
        min_idx = i;
      }
    }
  }

  if (min_idx == -1) {
    RCLCPP_WARN(get_logger(), "No valid LiDAR data detected!");
    return;
  }

  float angle = scan->angle_min + scan->angle_increment * min_idx;
  while (angle > M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }

  auto obstacle_msg = std_msgs::msg::Bool();
  obstacle_msg.data = distance_min < min_distance_;
  obstacle_pub_->publish(obstacle_msg);

  RCLCPP_INFO(get_logger(), "Laser at %f meters, angle: %f", distance_min, angle);

  switch (current_state_)
  {
    case State::FOLLOW_WALL:
      if (distance_min < wall_distance_ - 0.1f) {
        move_robot(0.2f, 0.2f);  // Gira hacia la pared
      } else if (distance_min > wall_distance_ + 0.1f) {
        move_robot(0.2f, -0.2f); // Gira alejándose de la pared
      } else {
        move_robot(0.2f, 0.0f);  // Mantiene la distancia de la pared
      }
  
      if (distance_min < min_distance_) {
        current_state_ = State::AVOID_OBSTACLE;  // Cambia al estado de evitar obstáculos
      }
      break;
  
    case State::SEARCH_WALL:
      if (distance_min < 2.0f) {
        current_state_ = State::FOLLOW_WALL;  // Empieza a seguir la pared si la detecta
      } else {
        move_robot(0.2f, 0.0f);  // Sigue recto si no detecta pared
      }
      break;
  
    case State::AVOID_OBSTACLE:
      if (distance_min < min_distance_) {
        move_robot(0.2f, 0.7f);  // Esquiva el obstáculo más rápidamente
      } else {
        current_state_ = State::FOLLOW_WALL;  // Vuelve a seguir la pared cuando el obstáculo es superado
      }
      break;
  }
  
}

}  // namespace FollowWall
