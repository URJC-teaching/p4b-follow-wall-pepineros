#ifndef FOLLOWWALL__FOLLOWWALL_HPP_
#define FOLLOWWALL__FOLLOWWALL_HPP_

// Librerías necesarias
#include <memory>
#include "sensor_msgs/msg/laser_scan.hpp"  // Recibimos los datos del LiDAR
#include "std_msgs/msg/bool.hpp"           
#include "geometry_msgs/msg/twist.hpp" 
#include "rclcpp/rclcpp.hpp"            

namespace FollowWall
{

// Definimos los estados de la máquina de estados del robot
enum class Estado
{
  SIGUIENDO_PARED,  
  BUSCANDO_PARED,
  EVADE_OBSTACULO   
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

  // Publicador que indica si hay un obstáculo
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;

  // Publicador que envía comandos de movimiento al robot
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Parámetros para la distancia
  float min_distance_ {0.5};        // Distancia antes de cambiar al estado de esquivar obstaculo
  float distance_to_wall_ {1.0};    // Distancia a la pared para seguirla correctamente

  // Variables para las distancias mínimas detectadas en las zonas del LIDAR
  float min_dist {0.0f};            // Distancia mínima en la zona delantera
  float min_dist_left {0.0f};       // Distancia mínima en la zona izquierda

  // Estado actual del robot dentro de la máquina de estados
  Estado estado_actual {Estado::SIGUIENDO_PARED};
};

}  // namespace FollowWall

#endif  // FOLLOWWALL__FOLLOWWALL_HPP_
