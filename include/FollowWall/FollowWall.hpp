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
  // Constructor
  FollowWallNode();

private:
  // Función que maneja los datos del LIDAR
  void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);

  // Función que controla el movimiento del robot
  void move_robot(float linear, float angular);

  // Suscripción al LIDAR
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  // Publicador para el mensaje de obstáculo
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;

  // Publicador para enviar los comandos de movimiento del robot
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Parámetros del robot
  float min_distance_ {0.5f};  // Distancia mínima para detectar obstáculos
  float distance_to_wall_ {1.0f}; // Distancia a la que el robot debe estar de la pared (1 metro)

  // Enum para los posibles estados del robot
  enum class Estado { SIGUIENDO_PARED, BUSCANDO_PARED, EVADE_OBSTACULO };
  Estado estado_actual;  // Almacena el estado actual del robot
};

}  // namespace FollowWall

#endif  // FOLLOWWALL__FOLLOWWALL_HPP_
