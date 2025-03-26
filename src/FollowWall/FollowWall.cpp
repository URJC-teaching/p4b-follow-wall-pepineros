#include <memory>
#include <algorithm>
#include "FollowWall/FollowWall.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("obstacle", 100);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void 
FollowWallNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  // Encuentra la distancia mínima medida por el LIDAR
  int min_idx = std::min_element(scan->ranges.begin(), scan->ranges.end()) - scan->ranges.begin();
  float distance_min = scan->ranges[min_idx];

  // Inicializar mensajes de control
  auto obstacle_msg = std_msgs::msg::Bool();
  auto cmd_vel_msg = geometry_msgs::msg::Twist();

  // Dependiendo del estado actual, actuamos
  switch (estado_actual) {
    case Estado::SIGUIENDO_PARED:
      // Si la distancia a la pared es menor de 1 metro, seguimos la pared
      if (distance_min > distance_to_wall_) {
        cmd_vel_msg.linear.x = 0.2;  // Avanzar hacia adelante
        cmd_vel_msg.angular.z = 0.0; // Mantener la dirección
      } else {
        cmd_vel_msg.linear.x = 0.0;  // Detenerse
        cmd_vel_msg.angular.z = 0.5; // Girar a la izquierda
      }
      break;

    case Estado::BUSCANDO_PARED:
      // Si no hay pared cercana (más de 2 metros), continuamos recto
      if (distance_min > 2.0) {
        cmd_vel_msg.linear.x = 0.2;  // Avanzar hacia adelante
        cmd_vel_msg.angular.z = 0.0; // Mantener la dirección
      } else {
        estado_actual = Estado::SIGUIENDO_PARED; // Encontramos pared, cambiamos a seguir la pared
      }
      break;

    case Estado::EVADE_OBSTACULO:
      // Si el obstáculo está demasiado cerca, detenerse y girar
      if (distance_min < 0.5) {
        cmd_vel_msg.linear.x = 0.0;  // Detenerse
        cmd_vel_msg.angular.z = 0.5; // Girar a la izquierda
      }
      break;
  }

  // Publicar los comandos de movimiento
  cmd_vel_pub_->publish(cmd_vel_msg);

  // Si hay un obstáculo, publicamos un mensaje
  if (distance_min < 0.5) {
    obstacle_msg.data = true;
    estado_actual = Estado::EVADE_OBSTACULO; // Cambiar a estado de evasión
  } else {
    obstacle_msg.data = false;
    if (estado_actual == Estado::EVADE_OBSTACULO) {
      estado_actual = Estado::SIGUIENDO_PARED; // Volver a seguir la pared
    } else if (estado_actual == Estado::SIGUIENDO_PARED) {
      estado_actual = Estado::BUSCANDO_PARED; // Si está siguiendo la pared, buscar otra pared si está lejos
    }
  }

  obstacle_pub_->publish(obstacle_msg);
}

}  // namespace FollowWall
