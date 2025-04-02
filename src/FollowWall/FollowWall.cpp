#include <memory>
#include <algorithm>
#include "FollowWall/FollowWall.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace FollowWall
{

using std::placeholders::_1;

// Constructor de la clase FollowWallNode
FollowWallNode::FollowWallNode() : Node("follow_wall_node")
{
  // Declaramos y obtenemos los parámetros
  declare_parameter("min_distance", min_distance_);
  declare_parameter("distance_to_wall", distance_to_wall_);
  get_parameter("min_distance", min_distance_);
  get_parameter("distance_to_wall", distance_to_wall_);

  // Suscripción al topic "scan_raw" (Gazebo) o "scan_filtered" (kobuki)
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_raw", rclcpp::SensorDataQoS().reliable(),
    std::bind(&FollowWallNode::laser_callback, this, _1));

  // Publicador para enviar comandos de velocidad al robot en el topic "cmd_vel"
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

// Callback que procesa los datos del Lidar
void 
FollowWallNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  // RCLCPP_INFO(get_logger(), "Rango del Lidar (%li puntos)", scan->ranges.size());

  // Zona de delante del Lidar (de 0 a 19 grados y de 340 a 359 grados)
  float dist_delante = scan->range_max;
  for(int idx = 0; idx < 19; idx++) 
  {
    float dist = scan->ranges[idx];
    if (dist < dist_delante) dist_delante = dist;
  }
  for(int idx = 340; idx < 359; idx++)
  {
    float dist = scan->ranges[idx];
    if (dist < dist_delante) dist_delante = dist;
  }
  RCLCPP_INFO(get_logger(), "Distancia delante: %f", dist_delante);

  // Zona izquierda del Lidar (de 60 a 120 grados)
  float dist_izquierda = scan->range_max;
  for(int idx = 60; idx < 120; idx++)
  {
    float dist = scan->ranges[idx];
    if (dist < dist_izquierda) dist_izquierda = dist;
  }
  RCLCPP_INFO(get_logger(), "Distancia izquierda: %f", dist_izquierda);

  auto cmd_vel_msg = geometry_msgs::msg::Twist();

  // Máquina de estados para decidir la acción del kobuki
  switch (estado_actual) {
    case Estado::SIGUIENDO_PARED:
      if (dist_delante < min_distance_ + 0.2) {  // Si hay un obstáculo en frente
        estado_actual = Estado::EVADE_OBSTACULO;
      } else if (dist_izquierda > distance_to_wall_ + 0.2) {  // Si está demasiado lejos de la pared
        estado_actual = Estado::BUSCANDO_PARED;
      } else if (dist_izquierda < distance_to_wall_ - 0.2) {  // Si está demasiado cerca de la pared
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.2;
      } else {  // Si está a la distancia de 1 metro de la pared izquierda
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.0;
      }
      break;

    case Estado::BUSCANDO_PARED:
      if (dist_izquierda > distance_to_wall_ + 0.2) {
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.0;
      } else {
        estado_actual = Estado::SIGUIENDO_PARED;
      }
      break;

    case Estado::EVADE_OBSTACULO:
      if (dist_delante < min_distance_ + 0.2) {
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.5;
      } else {
        estado_actual = Estado::SIGUIENDO_PARED;
      }
      break;
  }

  // Publica el mensaje con los comandos de velocidad
  cmd_vel_pub_->publish(cmd_vel_msg);
}

}  // namespace FollowWall




