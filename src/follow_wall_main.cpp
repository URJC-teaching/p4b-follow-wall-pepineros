#include <memory>
#include "bumpgo/bumpgo_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // Iniciamos ros2
  rclcpp::init(argc, argv);

  // Creamos un nodo compartido (make_shared) de tipo "BumpGoBehavior"
  auto bumpgo_node = std::make_shared<bumpgo::BumpGoBehavior>();

  // Mantenemos el nodo en ejecución con el spin
  rclcpp::spin(bumpgo_node);

  // Finalizamos ROS2
  rclcpp::shutdown();
  
  return 0;
}