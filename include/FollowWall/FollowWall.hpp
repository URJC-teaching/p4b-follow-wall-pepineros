#ifndef FOLLOWWALL__FOLLOWWALL_HPP_
#define FOLLOWWALL__FOLLOWWALL_HPP_

// Librerías necesarias
#include <memory>
#include "sensor_msgs/msg/laser_scan.hpp"  
#include "geometry_msgs/msg/twist.hpp"    
#include "rclcpp/rclcpp.hpp"              

namespace FollowWall
{

// Definimos los estados de la máquina de estados del Kobuki
enum class Estado
{
  SIGUIENDO_PARED, BUSCANDO_PARED, EVADE_OBSTACULO
};

class FollowWallNode : public rclcpp::Node
{
public:
  // Constructor de la clase
  FollowWallNode(); 

private:
  // Callback que se ejecuta cuando se recibe un mensaje del Lidar
  void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);

  // Método para controlar el movimiento del Kobuki
  void move_robot(float linear, float angular);

  // Suscriptor que recibe los datos del Lidar
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  // Publicador que envía comandos de movimiento al Kobuki
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  ////// Parámetros para la distancia
  // Distancia mínima para esquivar un obstáculo
  float min_distance_ {0.5}; 
  
  // Distancia deseada a la pared izquierda
  float distance_to_wall_ {1.0};

  // Iniciamos min_dist y min_dist_left a 0.0
  float dist_delante {0.0};
  float dist_izquierda {0.0};


  // Iniciamos el estado actual dentro de la máquina de estados
  Estado estado_actual {Estado::SIGUIENDO_PARED};
};

}  // namespace FollowWall

#endif  // FOLLOWWALL__FOLLOWWALL_HPP_