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

// Constructor de la clase FollowWallNode
FollowWallNode::FollowWallNode()
: Node("follow_wall_node")  // Inicializamos el nodo con el nombre "follow_wall_node"
{
  // Declaramos y obtenemos los parámetros
  declare_parameter("min_distance", min_distance_);
  declare_parameter("distance_to_wall", distance_to_wall_);
  get_parameter("min_distance", min_distance_);
  get_parameter("distance_to_wall", distance_to_wall_);

  // Suscripción al topic "scan_filtered" (Gazebo)
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_filtered", rclcpp::SensorDataQoS().reliable(),
    std::bind(&FollowWallNode::laser_callback, this, _1));

  // Publicador para indicar si hay un obstáculo en el topic "obstacle"
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("obstacle", 100);

  // Publicador para enviar comandos de velocidad al robot en el topic "cmd_vel"
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

// Callback que procesa los datos del Lidar
void FollowWallNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  // Variables para almacenar las distancias mínimas por delante y por la izquierda
  float front_distance = std::numeric_limits<float>::infinity();
  float left_distance = std::numeric_limits<float>::infinity();

  // Definir los ángulos para la zona delantera y la zona izquierda
  int front_start_idx = scan->ranges.size() / 4;  // Comienza a 45 grados hacia la derecha
  int front_end_idx = 3 * scan->ranges.size() / 4;  // Termina a 45 grados hacia la izquierda
  int left_start_idx = scan->ranges.size() / 2;  // Comienza a 90 grados hacia la izquierda
  int left_end_idx = scan->ranges.size() / 2 + 10;  // Termina 10 grados a la izquierda del robot

  // Filtrar valores inválidos y obtener las distancias mínimas en las zonas especificadas
  for (int i = front_start_idx; i < front_end_idx; ++i) {
    if (std::isfinite(scan->ranges[i]) && scan->ranges[i] > 0.05) {
      if (scan->ranges[i] < front_distance) {
        front_distance = scan->ranges[i];
      }
    }
  }

  for (int i = left_start_idx; i < left_end_idx; ++i) {
    if (std::isfinite(scan->ranges[i]) && scan->ranges[i] > 0.05) {
      if (scan->ranges[i] < left_distance) {
        left_distance = scan->ranges[i];
      }
    }
  }

  // Mensajes que se publicarán
  auto obstacle_msg = std_msgs::msg::Bool();  // Para indicar si hay un obstáculo
  auto cmd_vel_msg = geometry_msgs::msg::Twist();  // Para controlar el movimiento del robot

  // Máquina de estados para decidir la acción del robot
  switch (estado_actual) {
    case Estado::SIGUIENDO_PARED:
      if (left_distance > distance_to_wall_ + 0.2) {  // Si está demasiado lejos de la pared
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = -0.1;  // Corrige ligeramente hacia la pared
      } else if (left_distance < distance_to_wall_ - 0.2) {  // Si está demasiado cerca de la pared
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.2;  // Se aleja ligeramente de la pared
      } else {  // Si está a la distancia correcta
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.0;  // Sigue recto
      }
      break;

    case Estado::BUSCANDO_PARED:
      if (front_distance > 2.0) {  // Si no hay pared cerca, sigue avanzando recto
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.0;
      } else {  // Si encuentra una pared, cambia al estado de seguir la pared
        estado_actual = Estado::SIGUIENDO_PARED;
      }
      break;

    case Estado::EVADE_OBSTACULO:
      if (front_distance < 1.0) {  // Si hay un obstáculo demasiado cerca por delante
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.5;  // Gira a la izquierda para evitarlo
      } else {  // Si el obstáculo ha sido evitado, vuelve a seguir la pared
        estado_actual = Estado::SIGUIENDO_PARED;
      }
      break;
  }

  // Publica el mensaje con los comandos de velocidad
  cmd_vel_pub_->publish(cmd_vel_msg);

  // Determina si hay un obstáculo demasiado cerca y lo publica
  obstacle_msg.data = (front_distance < min_distance_);
  if (obstacle_msg.data) {
    estado_actual = Estado::EVADE_OBSTACULO;  // Si hay obstáculo, cambia al estado de evasión
  }
  obstacle_pub_->publish(obstacle_msg);
}

}  // namespace FollowWall
