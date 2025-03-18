#include <memory>

#include "FollowWall/FollowWall.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"

namespace FollowWall
{

using std::placeholders::_1;

FollowWallNode::FollowWallNode()
: Node("follow_wall_node")
{
  declare_parameter("min_distance", min_distance_);
  get_parameter("min_distance", min_distance_);

  RCLCPP_INFO(get_logger(), "FollowWallNode set to %f m", min_distance_);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS().reliable(),
    std::bind(&FollowWallNode::laser_callback, this, _1));
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>(
    "obstacle", 100);
}

void

FollowWallNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  int min_idx = std::min_element(scan->ranges.begin(), scan->ranges.end()) - scan->ranges.begin();
  float distance_min = scan->ranges[min_idx];

  auto obstacle_msg = std_msgs::msg::Bool();

  if (distance_min < 0.5) {
    float angle = scan->angle_min + scan->angle_increment * min_idx;
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}

    RCLCPP_INFO(get_logger(), "Obstacle in (%f, %f)", distance_min, angle);

    obstacle_msg.data = true;
  } else {
    obstacle_msg.data = false;
  }

  obstacle_pub_->publish(obstacle_msg);
}

}  // namespace FollowWall
