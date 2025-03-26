#include <memory>
#include "FollowWall/FollowWall.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Crear el nodo para seguir la pared
  auto node_follow_wall = std::make_shared<FollowWall::FollowWallNode>();

  // Ejecutar el nodo
  rclcpp::spin(node_follow_wall);

  // Finalizar y cerrar el nodo
  rclcpp::shutdown();

  return 0;
}
