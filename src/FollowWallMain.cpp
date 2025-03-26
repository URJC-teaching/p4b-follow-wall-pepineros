#include <memory>

#include "FollowWall/FollowWall.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_detector = std::make_shared<FollowWall::FollowWallNode>();

  rclcpp::spin(node_detector);

  rclcpp::shutdown();
  return 0;
}