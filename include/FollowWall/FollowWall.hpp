#ifndef FOLLOWWALL__FOLLOWWALL_HPP_
#define FOLLOWWALL__FOLLOWWALL_HPP_

// Librerías necesarias
#include <memory>
#include "sensor_msgs/msg/laser_scan.hpp"  // Recibimos los datos del LiDAR
#include "geometry_msgs/msg/twist.hpp"    // Mensajes de velocidad
#include "rclcpp/rclcpp.hpp"              // Funcionalidades de ROS2

namespace FollowWall
{

// Definimos los estados de la máquina de estados del robot
enum class Estado
{
  SIGUIENDO_PARED,   // El robot sigue la pared izquierda
  BUSCANDO_PARED,    // El robot busca una pared cercana
  EVADE_OBSTACULO    // El robot esquiva un obstáculo
};

class FollowWallNode : public rclcpp::Node
{
public:
  // Constructor de la clase
  FollowWallNode(); 

private:
  // Callback que se ejecuta cuando se recibe un mensaje del LiDAR
  void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);

  // Método para controlar el movimiento del robot
  void move_robot(float linear, float angular);  // Movimiento del robot

  // Suscriptor que recibe los datos del Lidar
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  // Publicador que envía comandos de movimiento al robot
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Parámetros para la distancia
  float min_distance_ {0.5};        // Distancia mínima antes de esquivar un obstáculo
  float distance_to_wall_ {1.0};    // Distancia deseada a la pared izquierda

  // Variables para las distancias mínimas detectadas en las zonas del LIDAR
  float min_dist {0.0};            // Distancia mínima en la zona delantera
  float min_dist_left {0.0};       // Distancia mínima en la zona izquierda

  // Estado actual del robot dentro de la máquina de estados
  Estado estado_actual {Estado::SIGUIENDO_PARED};
};

}  // namespace FollowWall

#endif  // FOLLOWWALL__FOLLOWWALL_HPP_