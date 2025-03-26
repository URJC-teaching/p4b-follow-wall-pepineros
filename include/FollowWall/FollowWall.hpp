#ifndef FOLLOWWALL__FOLLOWWALL_HPP_
#define FOLLOWWALL__FOLLOWWALL_HPP_

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"  // Incluir para usar comandos de movimiento
#include "rclcpp/rclcpp.hpp"

namespace FollowWall
{

// Definición de los estados para la máquina de estados
enum class Estado
{
  SIGUIENDO_PARED,  // Seguir la pared a la izquierda
  BUSCANDO_PARED,   // Continuar recto hasta encontrar otra pared
  EVADE_OBSTACULO   // Evitar obstáculo por la izquierda
};

class FollowWallNode : public rclcpp::Node
{
public:
  FollowWallNode();

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);

  // Suscripción al LIDAR
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  // Publicador para indicar si hay un obstáculo
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;

  // Publicador para enviar comandos de movimiento
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Parámetro para la distancia mínima de detección de obstáculos
  float min_distance_ {0.5f};

  // Parámetro para la distancia a la que debe estar la pared (seguimiento)
  float distance_to_wall_ {1.0f};

  // Estado actual de la máquina de estados
  Estado estado_actual {Estado::SIGUIENDO_PARED}; // El estado inicial es SIGUIENDO_PARED
};

}  // namespace FollowWall

#endif  // FOLLOWWALL__FOLLOWWALL_HPP_
