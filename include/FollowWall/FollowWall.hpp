#ifndef FOLLOWWALL__FOLLOWWALL_HPP_
#define FOLLOWWALL__FOLLOWWALL_HPP_

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"

namespace FollowWall
{

class FollowWallNode : public rclcpp::Node
{
public:
  FollowWallNode();

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;

  float min_distance_ {0.5f};
};

}  // namespace FollowWall

#endif  // FOLLOWWALL__FOLLOWWALL_HPP_
