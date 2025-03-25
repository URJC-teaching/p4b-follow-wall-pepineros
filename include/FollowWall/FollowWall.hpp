#ifndef FOLLOWWALL__FOLLOWWALL_HPP_
#define FOLLOWWALL__FOLLOWWALL_HPP_

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace FollowWall
{

class FollowWallNode : public rclcpp::Node
{
public:
  FollowWallNode();

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);
  void move_robot(float linear, float angular);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  float min_distance_ {0.5f};  // Distancia mínima para detectar obstáculos
  float wall_distance_ {1.0f}; // Distancia deseada de la pared
  enum class State { FOLLOW_WALL, SEARCH_WALL, AVOID_OBSTACLE };
  State current_state_ {State::SEARCH_WALL};
};

}  // namespace FollowWall

#endif  // FOLLOWWALL__FOLLOWWALL_HPP_
